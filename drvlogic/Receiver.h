#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <algorithm>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <type_traits>

namespace slamtec {

inline constexpr std::size_t MaxNodes = 8192;
inline constexpr std::size_t MaxSamples = 360 * 4;
inline constexpr std::array<int, 4> BaudRates{115200, 256000, 460800, 1000000};
inline constexpr std::array<const char*, 4> BaudNames{"115200", "256000", "460800", "1000000"};
int baudFromMenu(int index) noexcept;

enum class Transport { Serial, Tcp, Udp };
enum class State { Idle, Connecting, Streaming, Backoff, Stopping };
const char* stateName(State state) noexcept;

struct Connection {
    Transport transport = Transport::Serial;
    std::string address;
    int baudOrPort = 1000000;
    bool standardMode = false;
    bool operator==(const Connection& other) const noexcept;
};

struct OutputSettings {
    int precision = 2;
    bool qualityCheck = false;
    float minimumMm = 0;
    float maximumMm = 40000;
    bool operator==(const OutputSettings& other) const noexcept;
};

struct Request {
    bool active = false;
    Connection connection;
    OutputSettings output;
};

struct Node {
    float angleDegrees = 0;
    float distanceMm = 0;
    std::uint8_t quality = 0;
    std::uint8_t flag = 0;
};

struct Scan {
    std::array<Node, MaxNodes> nodes{};
    std::size_t count = 0;
    bool capacityReached = false;
    std::chrono::steady_clock::time_point captured{};
};

struct DeviceInfo {
    std::string model;
    std::string serial;
    std::string firmware;
    std::string hardware;
    std::string scanModes;
    std::string scanMode;
    std::string motorControl;
};

struct Sample {
    float angle = 0;
    float distance = 0;
    float quality = 0;
    float flag = 0;
};

template<std::size_t Capacity> class Text {
public:
    Text() = default;
    explicit Text(const char* value) noexcept { *this = value; }
    Text& operator=(const char* value) noexcept {
        std::size_t count = 0;
        while (count + 1 < Capacity && value[count] != '\0') {
            bytes_[count] = value[count];
            ++count;
        }
        bytes_[count] = '\0';
        return *this;
    }
    Text& operator=(const std::string& value) noexcept {
        const auto count = std::min(value.size(), Capacity - 1);
        std::copy_n(value.data(), count, bytes_.data());
        bytes_[count] = '\0';
        return *this;
    }
    const char* c_str() const noexcept { return bytes_.data(); }
private:
    std::array<char, Capacity> bytes_{};
};
struct SnapshotDeviceInfo {
    Text<100> model;
    Text<33> serial;
    Text<32> firmware, hardware;
    Text<2112> scanModes;
    Text<64> scanMode;
    Text<64> motorControl;
    SnapshotDeviceInfo& operator=(const DeviceInfo& value) noexcept {
        model = value.model; serial = value.serial; firmware = value.firmware; hardware = value.hardware;
        scanModes = value.scanModes; scanMode = value.scanMode; motorControl = value.motorControl;
        return *this;
    }
};

struct Snapshot {
    std::array<Sample, MaxSamples> samples{};
    std::size_t sampleCount = 720;
    std::size_t nodeCount = 0;
    std::uint64_t sequence = 0;
    std::uint64_t generation = 0;
    std::chrono::steady_clock::time_point captured{};
    State state = State::Idle;
    SnapshotDeviceInfo device;
    Text<257> message{"Inactive"};
};
static_assert(std::is_trivially_copyable_v<Snapshot>, "Cook handoff must not allocate");

// Single producer (worker), single consumer (TD cook). Each side owns its slot;
// only the middle slot changes hands. No mutex, allocation, or wait in exchange.
template <typename T>
class LatestMailbox {
public:
    T& writing() noexcept { return slots_[writer_]; }
    bool publish() noexcept {
        const unsigned previous = middle_.exchange(writer_ | Dirty, std::memory_order_acq_rel);
        writer_ = previous & Index;
        return (previous & Dirty) != 0;
    }
    bool consume(T& destination) {
        if ((middle_.load(std::memory_order_acquire) & Dirty) == 0) return false;
        const unsigned previous = middle_.exchange(reader_, std::memory_order_acq_rel);
        reader_ = previous & Index;
        destination = slots_[reader_];
        return true;
    }
private:
    static constexpr unsigned Dirty = 4;
    static constexpr unsigned Index = 3;
    std::array<T, 3> slots_{};
    unsigned writer_ = 0;
    unsigned reader_ = 1;
    std::atomic<unsigned> middle_{2};
};

class Cancellation {
public:
    Cancellation(const std::atomic<bool>& shutdown, const std::atomic<std::uint64_t>& revision,
                 std::uint64_t expected) : shutdown_(shutdown), revision_(revision), expected_(expected) {}
    bool requested() const noexcept {
        return shutdown_.load(std::memory_order_acquire) ||
               revision_.load(std::memory_order_acquire) != expected_;
    }
private:
    const std::atomic<bool>& shutdown_;
    const std::atomic<std::uint64_t>& revision_;
    std::uint64_t expected_;
};

enum class ReadResult { Scan, Timeout, Error };

// All backend methods, including destruction, run on the one receiver worker.
// Implementations must use finite I/O waits and check cancellation during setup.
class Backend {
public:
    virtual ~Backend() = default;
    virtual bool connect(const Connection&, const Cancellation&, DeviceInfo&, std::string& error) = 0;
    virtual ReadResult read(Scan&, const Cancellation&, std::string& error) = 0;
    virtual void close() noexcept = 0;
};

struct Statistics {
    std::uint64_t scans = 0;
    std::uint64_t dropped = 0;
    std::uint64_t timeouts = 0;
    std::uint64_t errors = 0;
    std::uint64_t reconnects = 0;
    std::uint64_t capacityHits = 0;
    std::uint64_t invalidNodes = 0;
};

struct Timing {
    std::chrono::milliseconds firstScan{5000};
    std::chrono::milliseconds stalledScan{2000};
    std::chrono::milliseconds retryMinimum{250};
    std::chrono::milliseconds retryMaximum{4000};
};

std::size_t processScan(const Scan&, const OutputSettings&, Snapshot&) noexcept;

class Receiver {
public:
    using Factory = std::function<std::unique_ptr<Backend>()>;
    explicit Receiver(Factory factory, Timing timing = {});
    ~Receiver();
    Receiver(const Receiver&) = delete;
    Receiver& operator=(const Receiver&) = delete;
    // False means the short control mailbox was busy: retry next cook.
    bool request(const Request& desired);
    bool consume(Snapshot& snapshot) { return mailbox_.consume(snapshot); }
    std::uint64_t generation() const noexcept { return revision_.load(std::memory_order_acquire); }
    Statistics statistics() const noexcept;
private:
    void run() noexcept;
    void publish(State, const Request&, const DeviceInfo&, const std::string&, std::uint64_t,
                 const Scan* = nullptr);
    bool copyRequest(Request&, std::uint64_t&);

    Factory factory_;
    Timing timing_;
    std::mutex requestMutex_;
    std::condition_variable wake_;
    Request desired_;
    std::atomic<std::uint64_t> revision_{0};
    std::atomic<bool> shutdown_{false};
    LatestMailbox<Snapshot> mailbox_;
    std::atomic<std::uint64_t> scans_{0}, dropped_{0}, timeouts_{0}, errors_{0}, reconnects_{0};
    std::atomic<std::uint64_t> capacityHits_{0}, invalidNodes_{0};
    std::thread worker_;
};

std::unique_ptr<Backend> makeSdkBackend();

} // namespace slamtec
