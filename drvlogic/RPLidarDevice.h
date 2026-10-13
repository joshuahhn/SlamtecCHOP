#pragma once

#include "Receiver.h"
#include "sl_lidar_driver.h"

namespace slamtec {
// Internal seam: tests use an in-memory protocol channel, production uses the SDK's
// serial/TCP/UDP channel. Both exercise the same SDK setup and shutdown paths.
using ChannelFactory = std::function<std::unique_ptr<sl::IChannel>(const Connection&, const Cancellation&)>;
std::unique_ptr<Backend> makeSdkBackend(ChannelFactory);
std::unique_ptr<sl::IChannel> makeNetworkChannel(const Connection&, const Cancellation&);
} // namespace slamtec
