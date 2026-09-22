#include <fpvsim/bridge/betaflight.hpp>

#include <algorithm>
#include <cstring>
#include <numbers>

namespace fpvsim::bridge::betaflight {

namespace {
constexpr double kDegPerRad = 180.0 / std::numbers::pi;
}

proto::FdmPacket make_fdm_packet(const FdmInput& input) {
  const Eigen::Vector3d& w = input.angular_rate_frd;
  const Eigen::Vector3d& f = input.specific_force_frd;
  const Eigen::Quaterniond& q = input.q_ned_from_frd;
  const Eigen::Vector3d& v = input.velocity_ned;
  return proto::FdmPacket{
      .timestamp = input.sim_time_s,
      .imu_angular_velocity_rpy = {w.x(), w.y(), w.z()},
      .imu_linear_acceleration_xyz = {-f.x(), f.y(), f.z()},
      .imu_orientation_quat = {q.w(), q.x(), -q.y(), -q.z()},
      .velocity_xyz = {v.y(), v.x(), -v.z()},
      .position_xyz = {input.longitude_rad * kDegPerRad, input.latitude_rad * kDegPerRad,
                       input.altitude_m},
      .pressure = input.pressure_pa,
  };
}

proto::RcPacket make_rc_packet(double sim_time_s, const RcChannels& channels) {
  return proto::RcPacket{.timestamp = sim_time_s, .channels = channels};
}

MotorCommands decode_servo_packet(const proto::ServoPacket& packet) {
  MotorCommands commands{};
  for (std::size_t i = 0; i < kMotorCount; ++i) {
    commands.command[i] = std::clamp(static_cast<double>(packet.motor_speed[i]), 0.0, 1.0);
  }
  return commands;
}

BetaflightLink::BetaflightLink(const Endpoints& endpoints)
    : motor_socket_(net::UdpSocket::bound(endpoints.host, endpoints.pwm_port)),
      out_socket_(net::UdpSocket::unbound()),
      fdm_address_(net::make_address(endpoints.host, endpoints.fdm_port)),
      rc_address_(net::make_address(endpoints.host, endpoints.rc_port)) {}

void BetaflightLink::send_fdm(const FdmInput& input) noexcept {
  const proto::FdmPacket packet = make_fdm_packet(input);
  if (!out_socket_.send_to(as_bytes(packet), fdm_address_)) {
    ++counters_.send_failures;
  }
}

void BetaflightLink::send_rc(double sim_time_s, const RcChannels& channels) noexcept {
  const proto::RcPacket packet = make_rc_packet(sim_time_s, channels);
  if (!out_socket_.send_to(as_bytes(packet), rc_address_)) {
    ++counters_.send_failures;
  }
}

bool BetaflightLink::poll_motors(MotorCommands& latest) noexcept {
  std::array<std::byte, 64> buffer{};
  bool received = false;
  while (const auto size = motor_socket_.try_receive(buffer)) {
    if (*size != sizeof(proto::ServoPacket)) {
      ++counters_.malformed_packets;
      continue;
    }
    proto::ServoPacket packet{};
    std::memcpy(&packet, buffer.data(), sizeof(packet));
    latest = decode_servo_packet(packet);
    ++counters_.motor_packets;
    received = true;
  }
  return received;
}

}  // namespace fpvsim::bridge::betaflight
