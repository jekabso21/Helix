#include <fpvsim/net/udp_socket.hpp>

#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <utility>

namespace fpvsim::net {

sockaddr_in make_address(const std::string& host, std::uint16_t port) {
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(port);
  if (inet_pton(AF_INET, host.c_str(), &address.sin_addr) != 1) {
    throw std::runtime_error("invalid IPv4 address: " + host);
  }
  return address;
}

UdpSocket::UdpSocket(int fd) : fd_(fd) {}

UdpSocket UdpSocket::unbound() {
  const int fd = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
  if (fd < 0) {
    throw std::runtime_error(std::string("socket: ") + std::strerror(errno));
  }
  return UdpSocket(fd);
}

UdpSocket UdpSocket::bound(const std::string& host, std::uint16_t port) {
  UdpSocket sock = unbound();
  const sockaddr_in address = make_address(host, port);
  if (bind(sock.fd_, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) != 0) {
    throw std::runtime_error("bind " + host + ":" + std::to_string(port) + ": " +
                             std::strerror(errno));
  }
  return sock;
}

UdpSocket::~UdpSocket() {
  if (fd_ >= 0) {
    close(fd_);
  }
}

UdpSocket::UdpSocket(UdpSocket&& other) noexcept : fd_(std::exchange(other.fd_, -1)) {}

UdpSocket& UdpSocket::operator=(UdpSocket&& other) noexcept {
  if (this != &other) {
    if (fd_ >= 0) {
      close(fd_);
    }
    fd_ = std::exchange(other.fd_, -1);
  }
  return *this;
}

bool UdpSocket::send_to(std::span<const std::byte> data,
                        const sockaddr_in& destination) const noexcept {
  const ssize_t sent = sendto(fd_, data.data(), data.size(), MSG_DONTWAIT,
                              reinterpret_cast<const sockaddr*>(&destination), sizeof(destination));
  return sent >= 0 && std::cmp_equal(sent, data.size());
}

std::optional<std::size_t> UdpSocket::try_receive(std::span<std::byte> buffer) const noexcept {
  const ssize_t received = recv(fd_, buffer.data(), buffer.size(), MSG_DONTWAIT);
  if (received < 0) {
    return std::nullopt;
  }
  return static_cast<std::size_t>(received);
}

}  // namespace fpvsim::net
