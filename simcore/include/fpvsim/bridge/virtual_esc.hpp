#pragma once

#include <chrono>
#include <cstdint>
#include <string>

#include <fpvsim/net/udp_socket.hpp>
#include <fpvsim/sim/snapshot.hpp>

namespace fpvsim::bridge {

struct EscStats {
  std::uint64_t requests = 0;
  std::uint64_t answered = 0;
  std::uint64_t connects = 0;
  bool connected = false;
};

// Answers Betaflight's ESC telemetry requests (one motor index byte on UDP) with KISS frames on
// the ESC sensor UART; runs on the I/O thread, never blocks
class VirtualEsc {
 public:
  VirtualEsc(const std::string& host, std::uint16_t request_port, std::uint16_t uart_port);
  ~VirtualEsc();
  VirtualEsc(const VirtualEsc&) = delete;
  VirtualEsc& operator=(const VirtualEsc&) = delete;

  // Drains requests and answers them from the latest snapshot; reconnects the UART as needed
  void service(const sim::Snapshot* latest, double ambient_c) noexcept;
  [[nodiscard]] const EscStats& stats() const { return stats_; }

 private:
  bool ensure_connected() noexcept;
  void disconnect() noexcept;
  bool answer(std::uint8_t motor, const sim::Snapshot& s, double ambient_c) noexcept;

  net::UdpSocket requests_;
  std::string host_;
  std::uint16_t uart_port_;
  int uart_fd_ = -1;
  std::chrono::steady_clock::time_point next_connect_;
  EscStats stats_;
};

}  // namespace fpvsim::bridge
