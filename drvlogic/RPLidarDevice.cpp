#include "RPLidarDevice.h"

#include <algorithm>
#include <array>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace slamtec {
namespace {

constexpr sl_u32 QueryTimeoutMs = 500;
constexpr sl_u32 ReceiveTimeoutMs = 100;

std::string failure(const char* operation, sl_result result) {
    std::ostringstream message;
    message << operation << " failed (0x" << std::hex << result << ')';
    return message.str();
}

std::unique_ptr<sl::IChannel> openChannel(const Connection& config, const Cancellation& cancellation) {
    if (config.transport != Transport::Serial) return makeNetworkChannel(config, cancellation);
    sl::Result<sl::IChannel*> result = SL_RESULT_OPERATION_FAIL;
    switch (config.transport) {
    case Transport::Serial: result = sl::createSerialPortChannel(config.address, config.baudOrPort); break;
    case Transport::Tcp:
    case Transport::Udp: break;
    }
    return result ? std::unique_ptr<sl::IChannel>(*result) : nullptr;
}

class GuardedChannel final : public sl::ISerialPortChannel {
public:
    GuardedChannel(std::unique_ptr<sl::IChannel> channel, const Cancellation& cancellation)
        : channel_(std::move(channel)), cancellation_(cancellation) {}
    ~GuardedChannel() override { close(); }
    bool open() override { return !cancelled() && channel_->open(); }
    void close() override {
        if (!closed_) {
            channel_->close();
            closed_ = true;
        }
    }
    void flush() override { if (!cancelled()) channel_->flush(); }
    bool waitForData(std::size_t size, sl_u32 timeout, std::size_t* actualReady) override {
        if (actualReady) *actualReady = 0;
        return !cancelled() && channel_->waitForData(size, std::min(timeout, ReceiveTimeoutMs), actualReady);
    }
    sl_result waitForDataExt(std::size_t& hint, sl_u32 timeout) override {
        hint = 0;
        if (cancelled()) return SL_RESULT_OPERATION_FAIL;
        const sl_result result = channel_->waitForDataExt(hint, std::min(timeout, ReceiveTimeoutMs));
        hint = SL_IS_OK(result) ? std::min<std::size_t>(hint, 65536) : 0;
        return result;
    }
    int write(const void* data, std::size_t size) override {
        return cancelled() ? -1 : channel_->write(data, size);
    }
    int read(void* data, std::size_t size) override {
        return cancelled() ? 0 : std::max(0, channel_->read(data, size));
    }
    void clearReadCache() override { if (!cancelled()) channel_->clearReadCache(); }
    int getChannelType() override { return channel_->getChannelType(); }
    void setDTR(bool value) override {
        if (!cancelled() && getChannelType() == sl::CHANNEL_TYPE_SERIALPORT)
            static_cast<sl::ISerialPortChannel*>(channel_.get())->setDTR(value);
    }
    void beginTeardown() noexcept { teardown_.store(true); }
private:
    bool cancelled() const noexcept { return cancellation_.requested() && !teardown_.load(); }
    std::unique_ptr<sl::IChannel> channel_;
    Cancellation cancellation_;
    std::atomic<bool> teardown_{false};
    bool closed_ = false;
};

class SdkBackend final : public Backend {
public:
    explicit SdkBackend(ChannelFactory channelFactory) : channelFactory_(std::move(channelFactory)) {}
    ~SdkBackend() override { close(); }

    bool connect(const Connection& config, const Cancellation& cancellation, DeviceInfo& info,
                 std::string& error) override {
        auto channel = channelFactory_(config, cancellation);
        if (!channel) { error = "Cannot create channel"; return false; }
        channel_ = std::make_unique<GuardedChannel>(std::move(channel), cancellation);
        auto driver = sl::createLidarDriver();
        if (!driver) { error = "Cannot create SDK driver"; return false; }
        driver_.reset(*driver);
        auto check = [&](sl_result result, const char* operation) {
            if (cancellation.requested()) { error = "Cancelled"; return false; }
            if (SL_IS_FAIL(result)) { error = failure(operation, result); return false; }
            return true;
        };
        if (!check(driver_->connect(channel_.get()), "Connect")) return false;
        sl_lidar_response_device_info_t device{};
        if (!check(driver_->getDeviceInfo(device, QueryTimeoutMs), "Device info")) return false;
        modelMajor_ = device.model >> 4;
        driver_->getModelNameDescriptionString(info.model, false, &device);
        std::ostringstream serial;
        for (auto byte : device.serialnum) serial << std::hex << std::setw(2) << std::setfill('0') << int(byte);
        info.serial = serial.str();
        info.firmware = std::to_string(device.firmware_version >> 8) + "." +
                        std::to_string(device.firmware_version & 0xff);
        info.hardware = std::to_string(device.hardware_version);
        sl_lidar_response_device_health_t health{};
        if (!check(driver_->getHealth(health, QueryTimeoutMs), "Health")) return false;
        if (health.status == SL_LIDAR_STATUS_ERROR) {
            error = "Device reports health error " + std::to_string(health.error_code);
            return false; // Never silently reset hardware.
        }
        const sl_result motorResult = driver_->checkMotorCtrlSupport(motorSupport_, QueryTimeoutMs);
        if (cancellation.requested()) { error = "Cancelled"; return false; }
        if (SL_IS_FAIL(motorResult) && !(modelMajor_ >= 4 && modelMajor_ < 6)) {
            error = failure("Motor capability", motorResult);
            return false;
        }
        if (SL_IS_FAIL(motorResult)) motorSupport_ = sl::MotorCtrlSupportNone;
        if (motorSupport_ == sl::MotorCtrlSupportPwm) info.motorControl = "PWM";
        else if (motorSupport_ == sl::MotorCtrlSupportRpm) info.motorControl = "RPM";
        else info.motorControl = modelMajor_ < 4 ? "Legacy DTR" : "Device-managed (none)";

        std::vector<sl::LidarScanMode> modes;
        if (!check(driver_->getAllSupportedScanModes(modes, QueryTimeoutMs), "Scan modes")) return false;
        sl_u16 selected = 0;
        if (config.standardMode) {
            if (!modes.empty()) {
                const auto found = std::find_if(modes.begin(), modes.end(), [](const sl::LidarScanMode& mode) {
                    return mode.ans_type == SL_LIDAR_ANS_TYPE_MEASUREMENT;
                });
                if (found == modes.end()) { error = "Device has no standard scan mode"; return false; }
                selected = found->id;
            } // Empty mode list is the SDK's documented legacy firmware path.
        } else {
            if (!check(driver_->getTypicalScanMode(selected, QueryTimeoutMs), "Typical scan mode")) return false;
            if (!modes.empty() && std::none_of(modes.begin(), modes.end(), [selected](const sl::LidarScanMode& mode) {
                    return mode.id == selected;
                })) {
                error = "Typical scan mode is absent from the supported list";
                return false;
            }
        }
        for (const auto& mode : modes) {
            if (!info.scanModes.empty()) info.scanModes += " | ";
            info.scanModes += std::string(mode.scan_mode, std::find(mode.scan_mode,
                                         mode.scan_mode + sizeof(mode.scan_mode), '\0'));
        }
        sl::LidarScanMode used{};
        started_ = true; // Clean up even if setup fails after motor/scan startup begins.
        if (!check(driver_->startScanExpress(false, selected, 0, &used, QueryTimeoutMs), "Start scan")) return false;
        info.scanMode.assign(used.scan_mode, std::find(used.scan_mode, used.scan_mode + sizeof(used.scan_mode), '\0'));
        if (info.scanMode.empty()) info.scanMode = config.standardMode ? "Legacy standard" : "Legacy typical";
        return true;
    }

    ReadResult read(Scan& scan, const Cancellation& cancellation, std::string& error) override {
        scan.count = 0;
        if (cancellation.requested()) return ReadResult::Timeout;
        std::size_t count = nodes_.size(); // Capacity is input/output; reset every call.
        const sl_result result = driver_->grabScanDataHq(nodes_.data(), count, ReceiveTimeoutMs);
        if (result == SL_RESULT_OPERATION_TIMEOUT) return ReadResult::Timeout;
        if (SL_IS_FAIL(result)) { error = failure("Receive", result); return ReadResult::Error; }
        if (count > nodes_.size()) { error = "SDK returned an invalid node count"; return ReadResult::Error; }
        if (count == 0) return ReadResult::Timeout;
        scan.count = count;
        scan.capacityReached = count == nodes_.size();
        for (std::size_t i = 0; i < count; ++i) {
            scan.nodes[i] = {static_cast<float>(nodes_[i].angle_z_q14) * 90.0f / 16384.0f,
                             static_cast<float>(nodes_[i].dist_mm_q2) / 4.0f, nodes_[i].quality, nodes_[i].flag};
        }
        return ReadResult::Scan;
    }

    void close() noexcept override {
        if (!driver_) { channel_.reset(); return; }
        channel_->beginTeardown();
        if (started_) {
            // SDK command encoding can allocate. Still disconnect/join after a
            // failed STOP instead of terminating the host from this noexcept path.
            try { driver_->stop(QueryTimeoutMs); } catch (...) {}
            if (motorSupport_ == sl::MotorCtrlSupportRpm) {
                try { driver_->setMotorSpeed(0); } catch (...) {}
            }
            if (motorSupport_ == sl::MotorCtrlSupportNone && modelMajor_ < 4 &&
                channel_->getChannelType() == sl::CHANNEL_TYPE_SERIALPORT) channel_->setDTR(true);
        }
        driver_->disconnect(); // Joins SDK decoder/RX before closing the channel.
        driver_.reset();
        channel_.reset();
        started_ = false;
    }
private:
    ChannelFactory channelFactory_;
    std::unique_ptr<GuardedChannel> channel_;
    std::unique_ptr<sl::ILidarDriver> driver_;
    std::array<sl_lidar_response_measurement_node_hq_t, MaxNodes> nodes_{};
    sl::MotorCtrlSupport motorSupport_ = sl::MotorCtrlSupportNone;
    unsigned modelMajor_ = 0;
    bool started_ = false;
};

} // namespace

std::unique_ptr<Backend> makeSdkBackend(ChannelFactory channelFactory) {
    return std::make_unique<SdkBackend>(std::move(channelFactory));
}

std::unique_ptr<Backend> makeSdkBackend() { return makeSdkBackend(openChannel); }

} // namespace slamtec
