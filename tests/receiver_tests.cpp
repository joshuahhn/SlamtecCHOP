#include "drvlogic/RPLidarDevice.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <cstring>
#include <future>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

using namespace slamtec;
using namespace std::chrono_literals;
namespace {
void require(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}
template<class Predicate> void until(Predicate predicate, const char* message, std::chrono::milliseconds timeout = 2000ms) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!predicate()) {
        require(std::chrono::steady_clock::now() < deadline, message);
        std::this_thread::sleep_for(1ms);
    }
}
void submit(Receiver& receiver, const Request& request) {
    until([&] { return receiver.request(request); }, "control mailbox failed to accept request");
}
Request activeRequest() {
    Request request;
    request.active = true;
    request.connection.address = "fake";
    return request;
}
void bounds() {
    for (int i = 0; i < 4; ++i) require(baudFromMenu(i) == std::stoi(BaudNames[static_cast<std::size_t>(i)]), "baud menu drift");
    require(baudFromMenu(-1) == 0 && baudFromMenu(4) == 0, "baud bounds");
    Scan scan;
    scan.count = 7;
    scan.nodes[0] = {359.99f, 1234.25f, 200, 3};
    scan.nodes[1] = {360, 1, 0, 0};
    scan.nodes[2] = {-1, 1, 0, 0};
    scan.nodes[3] = {std::numeric_limits<float>::quiet_NaN(), 1, 0, 0};
    scan.nodes[4] = {20, std::numeric_limits<float>::infinity(), 0, 0};
    scan.nodes[5] = {30, -1, 0, 0};
    scan.nodes[6] = {0, 12, 100, 1};
    OutputSettings settings;
    Snapshot snapshot;
    settings.precision = 99;
    require(processScan(scan, settings, snapshot) == 5 && snapshot.sampleCount == MaxSamples, "malformed scan bounds");
    require(snapshot.samples[1439].distance == 1234.25f, "millimetre precision changed");
    settings.qualityCheck = true;
    processScan(scan, settings, snapshot);
    require(snapshot.samples[0].distance == 0 && snapshot.samples[1439].distance == 1234.25f, "quality thresholds changed");
    settings.precision = -1;
    settings.minimumMm = 2000;
    processScan(scan, settings, snapshot);
    require(snapshot.sampleCount == 360 && snapshot.samples[359].distance == 0, "precision/filter bounds");
    scan.count = MaxNodes + 1;
    processScan(scan, settings, snapshot); // capacity bound, covered by sanitizer
}
void mailbox() {
    struct Packet { std::uint64_t sequence = 0; std::array<std::uint64_t, 128> values{}; };
    LatestMailbox<Packet> exchange;
    for (std::uint64_t i = 1; i <= 100; ++i) {
        exchange.writing().sequence = i;
        exchange.writing().values.fill(i);
        require(exchange.publish() == (i != 1), "overwrite reporting");
    }
    Packet packet;
    require(exchange.consume(packet) && packet.sequence == 100 && !exchange.consume(packet), "latest overwrite");
    std::atomic<bool> done{false};
    std::thread producer([&] {
        for (std::uint64_t i = 101; i <= 200000; ++i) {
            exchange.writing().sequence = i;
            exchange.writing().values.fill(i);
            exchange.publish();
        }
        done.store(true);
    });
    bool consistent = true;
    std::uint64_t last = 100;
    while (!done.load() || last < 200000) {
        if (!exchange.consume(packet)) { std::this_thread::yield(); continue; }
        consistent &= packet.sequence > last;
        for (auto value : packet.values) consistent &= value == packet.sequence;
        last = packet.sequence;
    }
    producer.join();
    require(consistent, "torn or non-monotonic concurrent snapshot");
}
struct FakeState {
    // 0 scans, 1 blocked connect, 2 blocked read, 3 failed connect, 4 timeouts, 5 oversized scan
    std::atomic<int> mode{0}, created{0}, destroyed{0}, closed{0}, connected{0}, reading{0}, live{0};
    std::atomic<bool> affinity{true};
};
class FakeBackend final : public Backend {
public:
    explicit FakeBackend(std::shared_ptr<FakeState> state) : state_(std::move(state)), owner_(std::this_thread::get_id()) {
        ++state_->created;
        if (++state_->live != 1) state_->affinity = false;
    }
    ~FakeBackend() override { check(); --state_->live; ++state_->destroyed; }
    bool connect(const Connection&, const Cancellation& cancellation, DeviceInfo&, std::string& error) override {
        check(); ++state_->connected;
        while (state_->mode == 1 && !cancellation.requested()) std::this_thread::sleep_for(1ms);
        if (state_->mode == 3) { error = "fake failure"; return false; }
        return !cancellation.requested();
    }
    ReadResult read(Scan& scan, const Cancellation& cancellation, std::string&) override {
        check(); ++state_->reading;
        while (state_->mode == 2 && !cancellation.requested()) std::this_thread::sleep_for(1ms);
        std::this_thread::sleep_for(1ms);
        scan.count = 1;
        scan.nodes[0] = {90, 1000, 200, 3};
        if (state_->mode == 4 || cancellation.requested()) return ReadResult::Timeout;
        if (state_->mode == 5) scan.count = MaxNodes + 1;
        scan.capacityReached = true;
        return ReadResult::Scan;
    }
    void close() noexcept override { check(); ++state_->closed; }
private:
    void check() noexcept { if (owner_ != std::this_thread::get_id()) state_->affinity = false; }
    std::shared_ptr<FakeState> state_;
    std::thread::id owner_;
};
Receiver::Factory factory(const std::shared_ptr<FakeState>& state) {
    return [state] { return std::make_unique<FakeBackend>(state); };
}
void lifecycle() {
    auto state = std::make_shared<FakeState>();
    for (int i = 0; i < 1000; ++i) { Receiver inactive(factory(state)); }
    require(state->created == 0, "inactive construction touched backend");
    {
        Receiver receiver(factory(state), {25ms, 25ms, 10ms, 40ms});
        auto request = activeRequest();
        submit(receiver, request);
        Snapshot frame;
        until([&] { receiver.consume(frame); return frame.state == State::Streaming; }, "no fake scan");
        const auto generation = receiver.generation();
        request.output.precision = 4;
        request.output.minimumMm = 2000;
        submit(receiver, request);
        until([&] { receiver.consume(frame); return frame.sampleCount == 1440 && frame.state == State::Streaming; }, "output update");
        require(frame.samples[360].distance == 0 && state->created == 1 && receiver.generation() == generation, "output edit reconnected or failed filter");
        request.connection.address = "replacement";
        submit(receiver, request);
        until([&] { receiver.consume(frame); return frame.generation == receiver.generation() && frame.state == State::Streaming; }, "generation replacement");
        require(frame.generation != generation && state->created == 2 && state->destroyed == 1, "old session not retired");
        state->mode = 4;
        until([&] { return receiver.statistics().reconnects > 1; }, "stall did not reconnect");
        const auto scans = receiver.statistics().scans;
        std::this_thread::sleep_for(15ms);
        require(receiver.statistics().scans == scans, "timeout buffer published");
        require(receiver.statistics().capacityHits > 0, "capacity diagnostics missing");
        request.active = false;
        submit(receiver, request);
        until([&] { receiver.consume(frame); return frame.state == State::Idle; }, "stop failed");
    }
    require(state->affinity && state->live == 0 && state->destroyed == state->created && state->closed == state->created, "worker ownership or cleanup");
    for (int mode : {1, 2, 3, 5}) {
        state = std::make_shared<FakeState>(); state->mode = mode;
        const auto start = std::chrono::steady_clock::now();
        {
            Receiver receiver(factory(state), {20ms, 20ms, 10ms, 20ms});
            auto request = activeRequest(); submit(receiver, request);
            until([&] { return mode == 2 ? state->reading > 0 : state->connected > 0; }, "blocked stage not entered");
            if (mode == 2) {
                std::vector<double> durations;
                Snapshot frame;
                for (int i = 0; i < 10000; ++i) {
                    const auto before = std::chrono::steady_clock::now();
                    receiver.request(request); receiver.consume(frame);
                    durations.push_back(std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - before).count());
                }
                std::sort(durations.begin(), durations.end());
                std::cout << "Synthetic blocked-read handoff p95/p99: " << durations[9500] << '/' << durations[9900] << " us (not C1 performance)\n";
                require(durations[9900] < 10000, "handoff waited for blocked I/O");
            }
            request.connection.address = "next";
            submit(receiver, request);
            until([&] { return state->created >= 2; }, "cancel/reconnect failed");
        }
        require(std::chrono::steady_clock::now() - start < 1000ms && state->affinity && state->live == 0, "blocked destruction/ownership");
    }
}

template<class T> void append(std::vector<sl_u8>& bytes, const T& value) {
    const auto* begin = reinterpret_cast<const sl_u8*>(&value);
    bytes.insert(bytes.end(), begin, begin + sizeof(value));
}
struct ProtocolState {
    std::mutex mutex;
    std::condition_variable wake;
    std::deque<sl_u8> incoming;
    std::vector<sl_u8> commands;
    std::vector<bool> dtr;
    std::vector<sl_u16> speeds;
    sl_u8 model = 0x41;
    bool pwm = false, openOk = true, silent = false, shortWrite = false, shortSpeed = false, legacy = false, badMac = false;
    bool malformedCount = false, invalidTypical = false, negativeRead = false;
    std::atomic<int> opens{0}, closes{0}, destroys{0};
    sl_u16 selected = 65535;
    sl_u32 maxWait = 0;
    bool streamStarted = false;
};
class ProtocolChannel final : public sl::ISerialPortChannel {
public:
    explicit ProtocolChannel(std::shared_ptr<ProtocolState> state) : state_(std::move(state)) {}
    ~ProtocolChannel() override { ++state_->destroys; }
    bool open() override { ++state_->opens; return state_->openOk; }
    void close() override { ++state_->closes; }
    void flush() override {}
    void clearReadCache() override {}
    int getChannelType() override { return sl::CHANNEL_TYPE_SERIALPORT; }
    void setDTR(bool value) override { std::lock_guard<std::mutex> lock(state_->mutex); state_->dtr.push_back(value); }
    bool waitForData(std::size_t, sl_u32 timeout, std::size_t* actual) override {
        std::size_t hint = 0; const auto result = waitForDataExt(hint, timeout);
        if (actual) *actual = hint; return SL_IS_OK(result);
    }
    sl_result waitForDataExt(std::size_t& hint, sl_u32 timeout) override {
        std::unique_lock<std::mutex> lock(state_->mutex);
        state_->maxWait = std::max(state_->maxWait, timeout);
        state_->wake.wait_for(lock, std::chrono::milliseconds(timeout), [&] { return !state_->incoming.empty(); });
        hint = state_->incoming.size();
        return hint ? SL_RESULT_OK : SL_RESULT_OPERATION_TIMEOUT;
    }
    int read(void* data, std::size_t size) override {
        std::lock_guard<std::mutex> lock(state_->mutex);
        if (state_->negativeRead) return -1;
        const auto count = std::min(size, state_->incoming.size());
        auto* bytes = static_cast<sl_u8*>(data);
        for (std::size_t i = 0; i < count; ++i) { bytes[i] = state_->incoming.front(); state_->incoming.pop_front(); }
        return static_cast<int>(count);
    }
    int write(const void* data, std::size_t size) override {
        std::lock_guard<std::mutex> lock(state_->mutex);
        const auto* bytes = static_cast<const sl_u8*>(data);
        if (size < 2 || bytes[0] != 0xA5) return -1;
        const sl_u8 command = bytes[1];
        state_->commands.push_back(command);
        if (state_->shortWrite) return static_cast<int>(size - 1);
        if (state_->silent) return static_cast<int>(size);
        std::vector<sl_u8> payload;
        if (command == SL_LIDAR_CMD_GET_DEVICE_INFO) {
            sl_lidar_response_device_info_t info{};
            info.model = state_->model; info.firmware_version = state_->legacy ? 0x0110 : 0x0210;
            append(payload, info); reply(SL_LIDAR_ANS_TYPE_DEVINFO, payload);
        } else if (command == SL_LIDAR_CMD_GET_DEVICE_HEALTH) {
            sl_lidar_response_device_health_t health{}; append(payload, health); reply(SL_LIDAR_ANS_TYPE_DEVHEALTH, payload);
        } else if (command == SL_LIDAR_CMD_GET_SAMPLERATE) {
            sl_lidar_response_sample_rate_t rate{};
            rate.std_sample_duration_us = 100; rate.express_sample_duration_us = 100;
            append(payload, rate); reply(SL_LIDAR_ANS_TYPE_SAMPLE_RATE, payload);
        } else if (command == SL_LIDAR_CMD_GET_ACC_BOARD_FLAG) {
            sl_lidar_response_acc_board_flag_t flag{};
            flag.support_flag = state_->pwm ? SL_LIDAR_RESP_ACC_BOARD_FLAG_MOTOR_CTRL_SUPPORT_MASK : 0;
            append(payload, flag); reply(SL_LIDAR_ANS_TYPE_ACC_BOARD_FLAG, payload);
        } else if (command == SL_LIDAR_CMD_GET_LIDAR_CONF && size >= 7) {
            sl_u32 type = 0; std::memcpy(&type, bytes + 3, sizeof(type)); append(payload, type);
            switch (type) {
            case SL_LIDAR_CONF_SCAN_MODE_COUNT: append(payload, static_cast<sl_u16>(state_->malformedCount ? 65535 : 3)); break;
            case SL_LIDAR_CONF_SCAN_MODE_TYPICAL: append(payload, static_cast<sl_u16>(state_->invalidTypical ? 99 : 2)); break;
            case SL_LIDAR_CONF_SCAN_MODE_US_PER_SAMPLE: append(payload, static_cast<sl_u32>(100 * 256)); break;
            case SL_LIDAR_CONF_SCAN_MODE_MAX_DISTANCE: append(payload, static_cast<sl_u32>(40 * 256)); break;
            case SL_LIDAR_CONF_SCAN_MODE_ANS_TYPE: payload.push_back(bytes[7] == 0 ? SL_LIDAR_ANS_TYPE_MEASUREMENT : SL_LIDAR_ANS_TYPE_MEASUREMENT_DENSE_CAPSULED); break;
            case SL_LIDAR_CONF_SCAN_MODE_NAME: payload.insert(payload.end(), 64, 'N'); break; // deliberately no NUL
            case SL_LIDAR_CONF_LIDAR_MAC_ADDR:
                if (state_->badMac) payload.insert(payload.end(), 64, 0);
                break;
            case SL_LIDAR_CONF_DESIRED_ROT_FREQ: {
                if (state_->shortSpeed) payload.push_back(0);
                else { sl_lidar_response_desired_rot_speed_t speed{}; speed.rpm = 620; speed.pwm_ref = 660; append(payload, speed); }
                break;
            }
            default: break;
            }
            reply(SL_LIDAR_ANS_TYPE_GET_LIDAR_CONF, payload);
        } else if ((command == SL_LIDAR_CMD_SET_MOTOR_PWM || command == SL_LIDAR_CMD_HQ_MOTOR_SPEED_CTRL) && size >= 5) {
            sl_u16 speed = 0; std::memcpy(&speed, bytes + 3, sizeof(speed)); state_->speeds.push_back(speed);
        } else if (command == SL_LIDAR_CMD_EXPRESS_SCAN && size >= 4) {
            state_->selected = bytes[3];
        }
        return static_cast<int>(size);
    }
    void reply(sl_u8 type, const std::vector<sl_u8>& payload) {
        std::vector<sl_u8> bytes{0xA5, 0x5A}; append(bytes, static_cast<sl_u32>(payload.size())); bytes.push_back(type);
        bytes.insert(bytes.end(), payload.begin(), payload.end());
        state_->incoming.insert(state_->incoming.end(), bytes.begin(), bytes.end()); state_->wake.notify_all();
    }
    static void injectScan(const std::shared_ptr<ProtocolState>& state, std::size_t count) {
        std::lock_guard<std::mutex> lock(state->mutex);
        std::vector<sl_u8> bytes;
        if (!state->streamStarted) {
            bytes = {0xA5, 0x5A};
            append(bytes, static_cast<sl_u32>(5 | (1u << 30)));
            bytes.push_back(SL_LIDAR_ANS_TYPE_MEASUREMENT);
            state->streamStarted = true;
        }
        for (std::size_t i = 0; i <= count; ++i) {
            sl_lidar_response_measurement_node_t node{};
            node.sync_quality = static_cast<sl_u8>(200 | (i == 0 || i == count ? 1 : 2));
            node.angle_q6_checkbit = static_cast<sl_u16>(((i % 360) * 64 << 1) | 1);
            node.distance_q2 = 4937; // 1234.25 mm
            append(bytes, node);
        }
        state->incoming.insert(state->incoming.end(), bytes.begin(), bytes.end()); state->wake.notify_all();
    }
private:
    std::shared_ptr<ProtocolState> state_;
};
void protocol() {
    std::atomic<bool> shutdown{false}; std::atomic<std::uint64_t> revision{0};
    Cancellation cancellation(shutdown, revision, 0);
    Connection config; config.address = "in-memory";
    for (int scenario = 0; scenario < 14; ++scenario) {
        auto state = std::make_shared<ProtocolState>();
        if (scenario == 1) { state->model = 0x11; state->legacy = true; config.standardMode = true; }
        else config.standardMode = false;
        if (scenario == 2 || scenario == 4) { state->model = 0x21; state->pwm = true; }
        if (scenario == 3) state->model = 0x81;
        if (scenario == 4) state->shortSpeed = true;
        if (scenario == 5) state->malformedCount = true;
        if (scenario == 6) state->invalidTypical = true;
        if (scenario == 7) state->openOk = false;
        if (scenario == 8) state->silent = true;
        if (scenario == 9) state->shortWrite = true;
        if (scenario == 10) state->negativeRead = true;
        if (scenario == 11) config.standardMode = true;
        if (scenario == 12) { state->model = 0x11; state->legacy = true; }
        if (scenario == 13) { state->model = 0x81; state->badMac = true; }
        const auto before = std::chrono::steady_clock::now();
        auto backend = makeSdkBackend([state](const Connection&, const Cancellation&) { return std::make_unique<ProtocolChannel>(state); });
        DeviceInfo info; std::string error;
        const bool connected = backend->connect(config, cancellation, info, error);
        if (scenario >= 5 && scenario <= 10) require(!connected, "malformed/open/timeout/partial-write path accepted");
        else {
            require(connected, error.c_str());
            require(info.scanMode.size() <= 63 && info.scanModes.size() <= 3 * 66, "mode names not terminated/bounded");
            if (!config.standardMode) require(state->selected == (state->legacy ? 0 : 2), "typical scan mode hardcoded");
            Scan scan; scan.count = MaxNodes; scan.nodes[0].distanceMm = 999;
            require(backend->read(scan, cancellation, error) == ReadResult::Timeout && scan.count == 0, "SDK timeout buffer leaked");
            if (scenario == 11) {
                ProtocolChannel::injectScan(state, 360);
                require(backend->read(scan, cancellation, error) == ReadResult::Scan && scan.count == 360, "actual SDK normal scan decode");
                require(scan.nodes[0].distanceMm == 1234.25f, "SDK quarter-mm precision changed");
                ProtocolChannel::injectScan(state, MaxNodes + 20);
                until([&] { return backend->read(scan, cancellation, error) == ReadResult::Scan && scan.count == MaxNodes; }, "SDK scan capacity bound");
                require(scan.capacityReached, "SDK capacity diagnostic missing");
            }
        }
        backend->close(); backend.reset();
        require(state->destroys == 1 && state->closes == 1 && state->maxWait <= 100, "SDK channel cleanup/wait bound");
        if (scenario == 0 || scenario == 11) require(state->dtr.empty() && state->speeds.empty(), "C1 sent legacy motor control");
        if (scenario == 1 || scenario == 12) require(state->dtr == std::vector<bool>({false, true}), "A1 DTR lifecycle changed");
        if (scenario == 2) require(std::find(state->speeds.begin(), state->speeds.end(), 660) != state->speeds.end() && state->speeds.back() == 0, "A2 PWM lifecycle");
        if (scenario == 3) require(std::find(state->speeds.begin(), state->speeds.end(), 620) != state->speeds.end() && state->speeds.back() == 0, "S RPM lifecycle");
        if (scenario == 4) require(std::find(state->speeds.begin(), state->speeds.end(), 600) != state->speeds.end(), "short desired speed safe fallback");
        require(std::chrono::steady_clock::now() - before < 1600ms, "SDK setup timeout not propagated");
    }
    // Cancellation while an actual SDK query waits, with no serial hardware.
    auto state = std::make_shared<ProtocolState>(); state->silent = true;
    auto future = std::async(std::launch::async, [&] {
        auto backend = makeSdkBackend([state](const Connection&, const Cancellation&) { return std::make_unique<ProtocolChannel>(state); });
        DeviceInfo info; std::string error; backend->connect(config, cancellation, info, error); backend->close();
    });
    until([&] { return state->opens > 0; }, "SDK cancellation setup");
    ++revision;
    require(future.wait_for(800ms) == std::future_status::ready, "SDK query cancellation/cleanup exceeded bound");
    future.get();
}

#ifdef _WIN32
using TestSocket = SOCKET;
constexpr TestSocket BadSocket = INVALID_SOCKET;
void dispose(TestSocket socket) { closesocket(socket); }
#else
using TestSocket = int;
constexpr TestSocket BadSocket = -1;
void dispose(TestSocket socket) { ::close(socket); }
#endif
class Endpoint {
public:
    TestSocket socket = BadSocket;
    explicit Endpoint(int type) : socket(::socket(AF_INET, type, 0)) { require(socket != BadSocket, "loopback socket"); }
    ~Endpoint() { if (socket != BadSocket) dispose(socket); }
    Endpoint(const Endpoint&) = delete;
    Endpoint& operator=(const Endpoint&) = delete;
    int bindPort() {
        sockaddr_in address{}; address.sin_family = AF_INET; address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        require(::bind(socket, reinterpret_cast<const sockaddr*>(&address), static_cast<int>(sizeof(address))) == 0, "loopback bind");
#ifdef _WIN32
        int size = static_cast<int>(sizeof(address));
#else
        socklen_t size = sizeof(address);
#endif
        require(getsockname(socket, reinterpret_cast<sockaddr*>(&address), &size) == 0, "loopback port");
        return ntohs(address.sin_port);
    }
};
void network() {
#ifdef _WIN32
    WSADATA data{}; require(WSAStartup(MAKEWORD(2, 2), &data) == 0, "loopback WSAStartup");
#endif
    std::atomic<bool> shutdown{false}; std::atomic<std::uint64_t> revision{0}; Cancellation cancellation(shutdown, revision, 0);
    Connection config; config.address = "127.0.0.1";
    for (auto transport : {Transport::Tcp, Transport::Udp}) {
        const bool tcp = transport == Transport::Tcp;
        Endpoint server(tcp ? SOCK_STREAM : SOCK_DGRAM);
        config.transport = transport; config.baudOrPort = server.bindPort();
        if (tcp) require(listen(server.socket, 1) == 0, "loopback listen");
        auto channel = makeNetworkChannel(config, cancellation);
        require(channel->open(), "bounded network open");
        TestSocket peer = tcp ? ::accept(server.socket, nullptr, nullptr) : server.socket;
        require(peer != BadSocket, "loopback accept");
        require(channel->write("ping", 4) == 4, "network write length");
        sockaddr_storage sender{};
#ifdef _WIN32
        int senderSize = static_cast<int>(sizeof(sender));
#else
        socklen_t senderSize = sizeof(sender);
#endif
        std::array<char, 32> bytes{};
        const auto received = tcp ? ::recv(peer, bytes.data(), 32, 0) : ::recvfrom(peer, bytes.data(), 32, 0, reinterpret_cast<sockaddr*>(&sender), &senderSize);
        require(received == 4 && std::string(bytes.data(), 4) == "ping", "network bytes changed");
        const auto sent = tcp ? ::send(peer, "pong", 4, 0) : ::sendto(peer, "pong", 4, 0, reinterpret_cast<const sockaddr*>(&sender), senderSize);
        require(sent == 4, "loopback response");
        std::size_t hint = 0;
        require(SL_IS_OK(channel->waitForDataExt(hint, 100)) && hint <= 65536, "network bounded RX wait");
        require(channel->read(bytes.data(), bytes.size()) == 4 && std::string(bytes.data(), 4) == "pong", "network RX bytes");
        require(channel->waitForDataExt(hint, 1) == SL_RESULT_OPERATION_TIMEOUT && hint == 0, "network timeout");
        if (tcp) {
            dispose(peer);
            require(SL_IS_OK(channel->waitForDataExt(hint, 100)) && channel->read(bytes.data(), bytes.size()) == 0, "TCP disconnect detection");
        }
        channel->close(); channel->close();
    }
    config.address = "localhost";
    auto invalid = makeNetworkChannel(config, cancellation);
    require(!invalid->open(), "unbounded DNS path accepted");
    config.address = "127.0.0.1"; ++revision;
    auto cancelled = makeNetworkChannel(config, cancellation);
    require(!cancelled->open(), "cancelled network open");
#ifdef _WIN32
    WSACleanup();
#endif
}
} // namespace
int main() {
    try {
        bounds(); mailbox(); lifecycle(); protocol(); network();
        std::cout << "Receiver bounds, mailbox, lifecycle and SDK protocol tests passed\n";
        return 0;
    } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
