#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <fpvsim/net/udp_socket.hpp>

#include "bridge/betaflight/protocol_2026_6_2.hpp"

namespace fpvsim::bridge::betaflight {

namespace proto = v2026_6_2;

inline constexpr std::size_t kRcChannels = proto::kMaxRcChannels;
inline constexpr std::size_t kMotorCount = proto::kMotorSpeedCount;

using RcChannels = std::array<std::uint16_t, kRcChannels>;

// Physically correct values; the packet builder applies Betaflight's sign conventions
struct FdmInput {
  double sim_time_s;
  Eigen::Vector3d angular_rate_frd;
  Eigen::Vector3d specific_force_frd;
  Eigen::Quaterniond q_ned_from_frd;
  Eigen::Vector3d velocity_ned;
  double longitude_rad;
  double latitude_rad;
  double altitude_m;
  double pressure_pa;
};

struct MotorCommands {
  std::array<double, kMotorCount> command;  // 0..1
};

proto::FdmPacket make_fdm_packet(const FdmInput& input);
proto::RcPacket make_rc_packet(double sim_time_s, const RcChannels& channels);
MotorCommands decode_servo_packet(const proto::ServoPacket& packet);

template <typename Packet>
std::span<const std::byte> as_bytes(const Packet& packet) {
  return std::as_bytes(std::span<const Packet, 1>(&packet, 1));
}

struct Endpoints {
  std::string host;
  std::uint16_t pwm_port;
  std::uint16_t fdm_port;
  std::uint16_t rc_port;
};

struct LinkCounters {
  std::uint64_t motor_packets = 0;
  std::uint64_t malformed_packets = 0;
  std::uint64_t send_failures = 0;
};

class BetaflightLink {
 public:
  explicit BetaflightLink(const Endpoints& endpoints);

  void send_fdm(const FdmInput& input) noexcept;
  void send_rc(double sim_time_s, const RcChannels& channels) noexcept;
  // Drains all pending motor packets and keeps the newest; true if any arrived
  bool poll_motors(MotorCommands& latest) noexcept;
  [[nodiscard]] const LinkCounters& counters() const { return counters_; }

 private:
  net::UdpSocket motor_socket_;
  net::UdpSocket out_socket_;
  sockaddr_in fdm_address_;
  sockaddr_in rc_address_;
  LinkCounters counters_;
};

}  // namespace fpvsim::bridge::betaflight
