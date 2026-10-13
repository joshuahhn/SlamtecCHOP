#include "RPLidarDevice.h"

#include <algorithm>
#include <cerrno>
#include <climits>
#include <cstring>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace slamtec {
namespace {
#ifdef _WIN32
using Socket = SOCKET;
constexpr Socket InvalidSocket = INVALID_SOCKET;
int socketError() { return WSAGetLastError(); }
bool pending(int error) { return error == WSAEWOULDBLOCK || error == WSAEINPROGRESS; }
void closeSocket(Socket socket) { closesocket(socket); }
#else
using Socket = int;
constexpr Socket InvalidSocket = -1;
int socketError() { return errno; }
bool pending(int error) { return error == EAGAIN || error == EWOULDBLOCK || error == EINPROGRESS; }
void closeSocket(Socket socket) { ::close(socket); }
#endif

// Numeric IPv4/IPv6 only: DNS has no portable bounded cancellation contract.
// Nonblocking sockets keep connect/send/receive and SDK teardown finite.
class NetworkChannel final : public sl::IChannel {
public:
    NetworkChannel(const Connection& config, const Cancellation& cancellation)
        : config_(config), cancellation_(cancellation) {}
    ~NetworkChannel() override { close(); }
    bool open() override {
        close();
#ifdef _WIN32
        WSADATA data{};
        if (WSAStartup(MAKEWORD(2, 2), &data) != 0) return false;
        winsock_ = true;
#endif
        sockaddr_storage address{};
        int length = 0;
        auto* ipv4 = reinterpret_cast<sockaddr_in*>(&address);
        auto* ipv6 = reinterpret_cast<sockaddr_in6*>(&address);
        if (inet_pton(AF_INET, config_.address.c_str(), &ipv4->sin_addr) == 1) {
            ipv4->sin_family = AF_INET;
            ipv4->sin_port = htons(static_cast<unsigned short>(config_.baudOrPort));
            length = static_cast<int>(sizeof(sockaddr_in));
        } else if (inet_pton(AF_INET6, config_.address.c_str(), &ipv6->sin6_addr) == 1) {
            ipv6->sin6_family = AF_INET6;
            ipv6->sin6_port = htons(static_cast<unsigned short>(config_.baudOrPort));
            length = static_cast<int>(sizeof(sockaddr_in6));
        } else { close(); return false; }
        socket_ = ::socket(address.ss_family, config_.transport == Transport::Udp ? SOCK_DGRAM : SOCK_STREAM, 0);
        if (socket_ == InvalidSocket) { close(); return false; }
#ifdef _WIN32
        u_long nonblocking = 1;
        if (ioctlsocket(socket_, FIONBIO, &nonblocking) != 0) { close(); return false; }
#else
        if (fcntl(socket_, F_SETFL, O_NONBLOCK) != 0) { close(); return false; }
#ifdef __APPLE__
        int noSigpipe = 1;
        if (setsockopt(socket_, SOL_SOCKET, SO_NOSIGPIPE, &noSigpipe, sizeof(noSigpipe)) != 0) {
            close(); return false;
        }
#endif
#endif
        if (cancellation_.requested()) { close(); return false; }
        if (::connect(socket_, reinterpret_cast<const sockaddr*>(&address), length) != 0) {
            if (!pending(socketError())) { close(); return false; }
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
            while (!cancellation_.requested() && std::chrono::steady_clock::now() < deadline) {
                const int ready = wait(true, 20);
                if (ready < 0) break;
                if (ready == 0) continue;
                int error = 0;
#ifdef _WIN32
                int size = static_cast<int>(sizeof(error));
#else
                socklen_t size = sizeof(error);
#endif
                if (getsockopt(socket_, SOL_SOCKET, SO_ERROR, reinterpret_cast<char*>(&error), &size) == 0 && error == 0)
                    return true;
                break;
            }
            close(); return false;
        }
        return true;
    }
    void close() override {
        if (socket_ != InvalidSocket) { closeSocket(socket_); socket_ = InvalidSocket; }
#ifdef _WIN32
        if (winsock_) { WSACleanup(); winsock_ = false; }
#endif
    }
    void flush() override { clearReadCache(); }
    bool waitForData(std::size_t, sl_u32 timeout, std::size_t* actual) override {
        std::size_t hint = 0;
        const auto result = waitForDataExt(hint, timeout);
        if (actual) *actual = hint;
        return SL_IS_OK(result);
    }
    sl_result waitForDataExt(std::size_t& hint, sl_u32 timeout) override {
        hint = 0;
        const int ready = wait(false, std::min<sl_u32>(timeout, 100));
        if (ready < 0) return SL_RESULT_OPERATION_FAIL;
        if (ready == 0) return SL_RESULT_OPERATION_TIMEOUT;
        // Read up to a bounded maximum, including whole UDP datagrams.
        hint = 65536;
        return SL_RESULT_OK;
    }
    int write(const void* data, std::size_t size) override {
        if (size > INT_MAX || socket_ == InvalidSocket) return -1;
        const auto* bytes = static_cast<const char*>(data);
        std::size_t sent = 0;
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
        while (sent < size) {
#if defined(MSG_NOSIGNAL)
            constexpr int flags = MSG_NOSIGNAL;
#else
            constexpr int flags = 0;
#endif
            const auto result = ::send(socket_, bytes + sent, static_cast<int>(size - sent), flags);
            if (result > 0) {
                sent += static_cast<std::size_t>(result);
                if (config_.transport == Transport::Udp) break;
            } else if (result == 0 || !pending(socketError())) return -1;
            if (sent < size && (std::chrono::steady_clock::now() >= deadline || wait(true, 10) < 0)) return -1;
        }
        return sent == size ? static_cast<int>(sent) : -1;
    }
    int read(void* data, std::size_t size) override {
        if (socket_ == InvalidSocket) return -1;
        return static_cast<int>(::recv(socket_, static_cast<char*>(data), static_cast<int>(std::min<std::size_t>(size, INT_MAX)), 0));
    }
    void clearReadCache() override {
        std::array<char, 4096> discard{};
        // Bound draining even if a peer continuously sends.
        for (int i = 0; i < 16 && wait(false, 0) > 0; ++i)
            if (read(discard.data(), discard.size()) <= 0) break;
    }
    int getChannelType() override {
        return config_.transport == Transport::Udp ? sl::CHANNEL_TYPE_UDP : sl::CHANNEL_TYPE_TCP;
    }
private:
    int wait(bool writing, sl_u32 timeout) {
        if (socket_ == InvalidSocket) return -1;
#ifndef _WIN32
        if (socket_ >= FD_SETSIZE) return -1;
#endif
        fd_set descriptors;
        FD_ZERO(&descriptors);
        FD_SET(socket_, &descriptors);
        timeval duration{};
        duration.tv_sec = static_cast<decltype(duration.tv_sec)>(timeout / 1000);
        duration.tv_usec = static_cast<decltype(duration.tv_usec)>((timeout % 1000) * 1000);
        return ::select(
#ifdef _WIN32
            0,
#else
            socket_ + 1,
#endif
            writing ? nullptr : &descriptors, writing ? &descriptors : nullptr, nullptr, &duration);
    }
    Connection config_;
    Cancellation cancellation_;
    Socket socket_ = InvalidSocket;
#ifdef _WIN32
    bool winsock_ = false;
#endif
};
} // namespace
std::unique_ptr<sl::IChannel> makeNetworkChannel(const Connection& config, const Cancellation& cancellation) {
    return std::make_unique<NetworkChannel>(config, cancellation);
}
} // namespace slamtec
