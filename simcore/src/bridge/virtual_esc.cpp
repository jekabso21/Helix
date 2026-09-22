#include <fpvsim/bridge/virtual_esc.hpp>

#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <cstddef>

#include <fpvsim/bridge/kiss.hpp>

namespace fpvsim::bridge {

namespace {

constexpr auto kReconnectDelay = std::chrono::seconds(1);

}  // namespace

VirtualEsc::VirtualEsc(const std::string& host, std::uint16_t request_port, std::uint16_t uart_port)
    : requests_(net::UdpSocket::bound(host, request_port)),
      host_(host),
      uart_port_(uart_port),
      next_connect_(std::chrono::steady_clock::now()) {}

VirtualEsc::~VirtualEsc() { disconnect(); }

void VirtualEsc::disconnect() noexcept {
  if (uart_fd_ >= 0) {
    close(uart_fd_);
    uart_fd_ = -1;
  }
  stats_.connected = false;
}

bool VirtualEsc::ensure_connected() noexcept {
  if (uart_fd_ >= 0) {
    return true;
  }
  const auto now = std::chrono::steady_clock::now();
  if (now < next_connect_) {
    return false;
  }
  next_connect_ = now + kReconnectDelay;
  const int fd = socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0);
  if (fd < 0) {
    return false;
  }
  const sockaddr_in address = net::make_address(host_, uart_port_);
  // blocking connect to a local listener returns at once; a refused port fails at once too
  if (connect(fd, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) != 0) {
    close(fd);
    return false;
  }
  const int one = 1;
  setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
  uart_fd_ = fd;
  stats_.connected = true;
  ++stats_.connects;
  return true;
}

bool VirtualEsc::answer(std::uint8_t motor, const sim::Snapshot& s, double ambient_c) noexcept {
  if (motor >= s.motor_count) {
    return false;
  }
  // temperature: a placeholder that warms with load until an ESC thermal model exists
  const kiss::EscTelemetry telemetry{.temperature_c = ambient_c + s.motor_bus_current_a[motor],
                                     .voltage_v = s.battery_voltage_v,
                                     .current_a = s.motor_bus_current_a[motor],
                                     .consumption_mah = s.motor_consumed_mah[motor],
                                     .erpm = s.motor_erpm[motor]};
  const auto frame = kiss::encode_frame(telemetry);
  const ssize_t sent = send(uart_fd_, frame.data(), frame.size(), MSG_NOSIGNAL | MSG_DONTWAIT);
  if (sent != static_cast<ssize_t>(frame.size())) {
    disconnect();
    return false;
  }
  return true;
}

void VirtualEsc::service(const sim::Snapshot* latest, double ambient_c) noexcept {
  std::array<std::byte, 16> datagram{};
  while (const auto size = requests_.try_receive(datagram)) {
    if (*size == 0) {
      continue;
    }
    ++stats_.requests;
    if (latest == nullptr || !ensure_connected()) {
      continue;
    }
    if (answer(static_cast<std::uint8_t>(datagram[0]), *latest, ambient_c)) {
      ++stats_.answered;
    }
  }
  if (uart_fd_ >= 0) {
    // the UART never carries data towards the ESC; drop anything so the buffer cannot fill
    std::array<std::byte, 64> sink{};
    while (recv(uart_fd_, sink.data(), sink.size(), MSG_DONTWAIT) > 0) {
    }
  }
}

}  // namespace fpvsim::bridge
