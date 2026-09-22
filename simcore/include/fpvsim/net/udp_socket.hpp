#pragma once

#include <netinet/in.h>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>

namespace fpvsim::net {

sockaddr_in make_address(const std::string& host, std::uint16_t port);

// Non-blocking UDP socket; construction may throw (startup only), I/O never does
class UdpSocket {
 public:
  static UdpSocket bound(const std::string& host, std::uint16_t port);
  static UdpSocket unbound();
  ~UdpSocket();
  UdpSocket(UdpSocket&& other) noexcept;
  UdpSocket& operator=(UdpSocket&& other) noexcept;
  UdpSocket(const UdpSocket&) = delete;
  UdpSocket& operator=(const UdpSocket&) = delete;

  [[nodiscard]] bool send_to(std::span<const std::byte> data,
                             const sockaddr_in& destination) const noexcept;
  [[nodiscard]] std::optional<std::size_t> try_receive(std::span<std::byte> buffer) const noexcept;

 private:
  explicit UdpSocket(int fd);
  int fd_ = -1;
};

}  // namespace fpvsim::net
