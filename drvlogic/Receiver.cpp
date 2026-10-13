#include "Receiver.h"

#include <algorithm>
#include <cmath>
#include <exception>
#include <stdexcept>

namespace slamtec {

int baudFromMenu(int index) noexcept {
    return index >= 0 && index < static_cast<int>(BaudRates.size()) ? BaudRates[index] : 0;
}

const char* stateName(State state) noexcept {
    switch (state) {
    case State::Idle: return "Inactive";
    case State::Connecting: return "Connecting";
    case State::Streaming: return "Streaming";
    case State::Backoff: return "Retrying";
    case State::Stopping: return "Stopping";
    }
    return "Unknown";
}

bool Connection::operator==(const Connection& other) const noexcept {
    return transport == other.transport && address == other.address &&
           baudOrPort == other.baudOrPort && standardMode == other.standardMode;
}

bool OutputSettings::operator==(const OutputSettings& other) const noexcept {
    return precision == other.precision && qualityCheck == other.qualityCheck &&
           minimumMm == other.minimumMm && maximumMm == other.maximumMm;
}

std::size_t processScan(const Scan& scan, const OutputSettings& settings, Snapshot& snapshot) noexcept {
    const int precision = std::clamp(settings.precision, 1, 4);
    const float angularPrecision = static_cast<float>(precision);
    snapshot.sampleCount = 360 * static_cast<std::size_t>(precision);
    for (std::size_t i = 0; i < snapshot.sampleCount; ++i) {
        snapshot.samples[i] = {static_cast<float>(i) / angularPrecision, 0, 0, 0};
    }
    std::size_t invalid = 0;
    for (std::size_t i = 0; i < std::min(scan.count, MaxNodes); ++i) {
        const Node& node = scan.nodes[i];
        if (!std::isfinite(node.angleDegrees) || node.angleDegrees < 0 || node.angleDegrees >= 360 ||
            !std::isfinite(node.distanceMm) || node.distanceMm < 0) {
            ++invalid;
            continue;
        }
        const auto index = static_cast<std::size_t>(node.angleDegrees * angularPrecision);
        if (index >= snapshot.sampleCount) {
            ++invalid;
            continue;
        }
        if (node.distanceMm < settings.minimumMm || node.distanceMm > settings.maximumMm) continue;
        // Preserve the original opt-in Quality Check thresholds and millimetre output.
        if (settings.qualityCheck && (node.flag < 2 || node.quality < 150)) continue;
        snapshot.samples[index] = {static_cast<float>(index) / angularPrecision, node.distanceMm,
                                  static_cast<float>(node.quality), static_cast<float>(node.flag)};
    }
    return invalid;
}

Receiver::Receiver(Factory factory, Timing timing) : factory_(std::move(factory)), timing_(timing) {
    if (!factory_ || timing_.firstScan.count() <= 0 || timing_.stalledScan.count() <= 0 ||
        timing_.retryMinimum.count() <= 0 || timing_.retryMaximum < timing_.retryMinimum) {
        throw std::invalid_argument("Invalid receiver factory or timing");
    }
    worker_ = std::thread(&Receiver::run, this);
}

Receiver::~Receiver() {
    {
        std::lock_guard<std::mutex> lock(requestMutex_);
        shutdown_.store(true, std::memory_order_release);
    }
    wake_.notify_all();
    if (worker_.joinable()) worker_.join();
}

bool Receiver::request(const Request& desired) {
    std::unique_lock<std::mutex> lock(requestMutex_, std::try_to_lock);
    if (!lock.owns_lock()) return false;
    const bool connectionChanged = desired.active != desired_.active ||
                                   !(desired.connection == desired_.connection);
    if (!connectionChanged && desired.output == desired_.output) return true;
    desired_ = desired;
    desired_.output.precision = std::clamp(desired.output.precision, 1, 4);
    if (!std::isfinite(desired_.output.minimumMm)) desired_.output.minimumMm = 0;
    if (!std::isfinite(desired_.output.maximumMm)) desired_.output.maximumMm = 40000;
    desired_.output.minimumMm = std::max(0.0f, desired_.output.minimumMm);
    desired_.output.maximumMm = std::max(desired_.output.minimumMm, desired_.output.maximumMm);
    if (connectionChanged) revision_.fetch_add(1, std::memory_order_release);
    lock.unlock();
    wake_.notify_all();
    return true;
}

bool Receiver::copyRequest(Request& request, std::uint64_t& generation) {
    std::lock_guard<std::mutex> lock(requestMutex_);
    if (shutdown_.load(std::memory_order_acquire)) return false;
    request = desired_;
    generation = revision_.load(std::memory_order_acquire);
    return true;
}

Statistics Receiver::statistics() const noexcept {
    return {scans_.load(), dropped_.load(), timeouts_.load(), errors_.load(), reconnects_.load(),
            capacityHits_.load(), invalidNodes_.load()};
}

void Receiver::publish(State state, const Request& request, const DeviceInfo& info,
                       const std::string& message, std::uint64_t generation, const Scan* scan) {
    Snapshot& snapshot = mailbox_.writing();
    snapshot.state = state;
    snapshot.generation = generation;
    snapshot.device = info;
    snapshot.message = message;
    snapshot.sequence = scans_.load();
    snapshot.nodeCount = scan ? scan->count : 0;
    snapshot.captured = scan ? scan->captured : std::chrono::steady_clock::time_point{};
    if (scan) {
        invalidNodes_.fetch_add(processScan(*scan, request.output, snapshot));
    } else {
        snapshot.sampleCount = 360 * static_cast<std::size_t>(request.output.precision);
        std::fill(snapshot.samples.begin(), snapshot.samples.end(), Sample{});
    }
    if (mailbox_.publish()) dropped_.fetch_add(1);
}

void Receiver::run() noexcept {
    std::unique_ptr<Backend> backend;
    auto close = [&] {
        if (backend) {
            backend->close();
            backend.reset();
        }
    };
    try {
        Request desired;
        DeviceInfo info;
        Scan scan;
        std::uint64_t generation = 0;
        std::uint64_t sessionGeneration = 0;
        bool attempted = false;
        bool received = false;
        auto lastData = std::chrono::steady_clock::now();
        auto retry = timing_.retryMinimum;

        while (copyRequest(desired, generation)) {
            if (backend && (generation != sessionGeneration || !desired.active)) {
                close();
                info = {};
                received = false;
            }
            if (!desired.active) {
                attempted = false;
                retry = timing_.retryMinimum;
                publish(State::Idle, desired, {}, "Inactive", generation);
                std::unique_lock<std::mutex> lock(requestMutex_);
                wake_.wait(lock, [&] { return shutdown_.load() || desired_.active; });
                continue;
            }
            Cancellation cancellation(shutdown_, revision_, generation);
            if (cancellation.requested()) continue;

            if (!backend) {
                const bool validBaud = std::find(BaudRates.begin(), BaudRates.end(),
                                                desired.connection.baudOrPort) != BaudRates.end();
                const bool validPort = desired.connection.baudOrPort > 0 &&
                                       desired.connection.baudOrPort <= 65535;
                if (desired.connection.address.empty() || desired.connection.address.size() > 255 ||
                    (desired.connection.transport == Transport::Serial ? !validBaud : !validPort)) {
                    publish(State::Backoff, desired, {}, "Invalid address, baud rate or network port", generation);
                    std::unique_lock<std::mutex> lock(requestMutex_);
                    wake_.wait(lock, [&] { return cancellation.requested(); });
                    continue;
                }
                if (attempted) reconnects_.fetch_add(1);
                attempted = true;
                sessionGeneration = generation;
                info = {};
                publish(State::Connecting, desired, info, "Connecting", generation);
                std::string error;
                bool connected = false;
                try {
                    backend = factory_();
                    connected = backend && backend->connect(desired.connection, cancellation, info, error);
                } catch (const std::exception& exception) {
                    error = exception.what();
                } catch (...) {
                    error = "Backend setup failed";
                }
                if (!connected || cancellation.requested()) {
                    close();
                    if (cancellation.requested()) {
                        retry = timing_.retryMinimum;
                        continue;
                    }
                    errors_.fetch_add(1);
                    publish(State::Backoff, desired, {}, error.empty() ? "Connection failed" : error, generation);
                    std::unique_lock<std::mutex> lock(requestMutex_);
                    wake_.wait_for(lock, retry, [&] { return cancellation.requested(); });
                    retry = std::min(retry * 2, timing_.retryMaximum);
                    continue;
                }
                received = false;
                lastData = std::chrono::steady_clock::now();
                publish(State::Connecting, desired, info, "Waiting for first scan", generation);
            }

            std::string error;
            ReadResult result = ReadResult::Error;
            scan.count = 0;
            scan.capacityReached = false;
            try {
                result = backend->read(scan, cancellation, error);
            } catch (const std::exception& exception) {
                error = exception.what();
            } catch (...) {
                error = "Backend receive failed";
            }
            if (cancellation.requested()) continue;
            if (result == ReadResult::Scan && (scan.count == 0 || scan.count > MaxNodes)) {
                error = "Invalid scan size";
                result = ReadResult::Error;
            }
            if (result == ReadResult::Scan) {
                scans_.fetch_add(1);
                if (scan.capacityReached) capacityHits_.fetch_add(1);
                received = true;
                retry = timing_.retryMinimum;
                lastData = scan.captured = std::chrono::steady_clock::now();
                publish(State::Streaming, desired, info,
                        scan.capacityReached ? "Scan capacity reached; data may be truncated" : "Streaming",
                        generation, &scan);
                continue;
            }
            if (result == ReadResult::Timeout) {
                timeouts_.fetch_add(1);
                const auto deadline = received ? timing_.stalledScan : timing_.firstScan;
                if (std::chrono::steady_clock::now() - lastData < deadline) continue;
                error = received ? "Scan stream stalled" : "No first scan received";
            }
            errors_.fetch_add(1);
            close();
            publish(State::Backoff, desired, info, error.empty() ? "Receive failed" : error, generation);
            std::unique_lock<std::mutex> lock(requestMutex_);
            wake_.wait_for(lock, retry, [&] { return cancellation.requested(); });
            retry = std::min(retry * 2, timing_.retryMaximum);
        }
    } catch (...) {
        // Do not propagate exceptions out of std::thread into the TD process.
        errors_.fetch_add(1);
    }
    close();
}

} // namespace slamtec
