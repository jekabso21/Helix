#include <gtest/gtest.h>

#include <cstring>
#include <numbers>
#include <string>
#include <string_view>

#include <fpvsim/bridge/betaflight.hpp>

namespace bf = fpvsim::bridge::betaflight;
using Eigen::Vector3d;

namespace {

bf::FdmInput sample_input() {
  return bf::FdmInput{.sim_time_s = 1.5,
                      .angular_rate_frd = Vector3d(0.1, 0.2, 0.3),
                      .specific_force_frd = Vector3d(1.0, 2.0, -9.80665),
                      .q_ned_from_frd = Eigen::Quaterniond(0.8, 0.1, 0.2, 0.3),
                      .velocity_ned = Vector3d(4.0, 5.0, -6.0),
                      .longitude_rad = 24.0 * std::numbers::pi / 180.0,
                      .latitude_rad = 56.0 * std::numbers::pi / 180.0,
                      .altitude_m = 12.5,
                      .pressure_pa = 101325.0};
}

std::string hex(std::span<const std::byte> bytes) {
  static constexpr std::string_view kDigits = "0123456789abcdef";
  std::string out;
  for (const std::byte b : bytes) {
    out += kDigits[std::to_integer<std::size_t>(b) >> 4];
    out += kDigits[std::to_integer<std::size_t>(b) & 0xF];
  }
  return out;
}

}  // namespace

TEST(BridgeTest, FdmPacketAppliesTheVerifiedSignConventions) {
  const bf::proto::FdmPacket packet = bf::make_fdm_packet(sample_input());
  EXPECT_DOUBLE_EQ(packet.timestamp, 1.5);
  EXPECT_DOUBLE_EQ(packet.imu_angular_velocity_rpy[0], 0.1);
  EXPECT_DOUBLE_EQ(packet.imu_angular_velocity_rpy[1], 0.2);
  EXPECT_DOUBLE_EQ(packet.imu_angular_velocity_rpy[2], 0.3);
  EXPECT_DOUBLE_EQ(packet.imu_linear_acceleration_xyz[0], -1.0);
  EXPECT_DOUBLE_EQ(packet.imu_linear_acceleration_xyz[1], 2.0);
  EXPECT_DOUBLE_EQ(packet.imu_linear_acceleration_xyz[2], -9.80665);
  EXPECT_DOUBLE_EQ(packet.imu_orientation_quat[0], 0.8);
  EXPECT_DOUBLE_EQ(packet.imu_orientation_quat[1], 0.1);
  EXPECT_DOUBLE_EQ(packet.imu_orientation_quat[2], -0.2);
  EXPECT_DOUBLE_EQ(packet.imu_orientation_quat[3], -0.3);
}

TEST(BridgeTest, FdmPacketSendsEnuVelocityAndGeodeticPosition) {
  const bf::proto::FdmPacket packet = bf::make_fdm_packet(sample_input());
  EXPECT_DOUBLE_EQ(packet.velocity_xyz[0], 5.0);     // east
  EXPECT_DOUBLE_EQ(packet.velocity_xyz[1], 4.0);     // north
  EXPECT_DOUBLE_EQ(packet.velocity_xyz[2], 6.0);     // up
  EXPECT_NEAR(packet.position_xyz[0], 24.0, 1e-12);  // longitude, degrees
  EXPECT_NEAR(packet.position_xyz[1], 56.0, 1e-12);  // latitude, degrees
  EXPECT_DOUBLE_EQ(packet.position_xyz[2], 12.5);
  EXPECT_DOUBLE_EQ(packet.pressure, 101325.0);
}

TEST(BridgeTest, FdmPacketGoldenBytes) {
  const bf::proto::FdmPacket packet = bf::make_fdm_packet(sample_input());
  const std::string bytes = hex(bf::as_bytes(packet));
  ASSERT_EQ(bytes.size(), 2 * 144U);
  EXPECT_EQ(bytes.substr(0, 16), "000000000000f83f");    // 1.5
  EXPECT_EQ(bytes.substr(64, 16), "000000000000f0bf");   // -1.0, acceleration x
  EXPECT_EQ(bytes.substr(272, 16), "00000000d0bcf840");  // 101325.0, pressure
}

TEST(BridgeTest, RcPacketCarriesChannelsInOrder) {
  bf::RcChannels channels{};
  channels.fill(1000);
  channels[2] = 1300;
  channels[4] = 2000;
  const bf::proto::RcPacket packet = bf::make_rc_packet(2.0, channels);
  EXPECT_DOUBLE_EQ(packet.timestamp, 2.0);
  EXPECT_EQ(packet.channels[2], 1300);
  EXPECT_EQ(packet.channels[4], 2000);
  EXPECT_EQ(packet.channels[15], 1000);
  EXPECT_EQ(bf::as_bytes(packet).size(), 40U);
}

TEST(BridgeTest, ServoPacketIsClampedToUnitRange) {
  bf::proto::ServoPacket packet{.motor_speed = {0.303F, -0.5F, 1.5F, 1.0F}};
  const bf::MotorCommands commands = bf::decode_servo_packet(packet);
  EXPECT_NEAR(commands.command[0], 0.303, 1e-6);
  EXPECT_DOUBLE_EQ(commands.command[1], 0.0);
  EXPECT_DOUBLE_EQ(commands.command[2], 1.0);
  EXPECT_DOUBLE_EQ(commands.command[3], 1.0);
}

namespace {

bf::Endpoints loopback() {
  return {.host = "127.0.0.1", .pwm_port = 19002, .fdm_port = 19003, .rc_port = 19004};
}

template <typename Predicate>
void spin_until(Predicate done) {
  for (int i = 0; i < 100000 && !done(); ++i) {
  }
}

}  // namespace

TEST(BridgeTest, LinkSendsFdmPacketsOverLoopback) {
  fpvsim::net::UdpSocket fdm_sink = fpvsim::net::UdpSocket::bound("127.0.0.1", 19003);
  bf::BetaflightLink link(loopback());
  link.send_fdm(sample_input());
  std::array<std::byte, 256> buffer{};
  std::optional<std::size_t> size;
  spin_until([&] {
    size = fdm_sink.try_receive(buffer);
    return size.has_value();
  });
  ASSERT_TRUE(size.has_value());
  EXPECT_EQ(size.value_or(0), sizeof(bf::proto::FdmPacket));
  EXPECT_EQ(link.counters().send_failures, 0U);
}

TEST(BridgeTest, LinkKeepsTheNewestMotorPacketAndCountsMalformedOnes) {
  bf::BetaflightLink link(loopback());
  fpvsim::net::UdpSocket motor_source = fpvsim::net::UdpSocket::unbound();
  const sockaddr_in pwm = fpvsim::net::make_address("127.0.0.1", 19002);
  const bf::proto::ServoPacket first{.motor_speed = {0.1F, 0.2F, 0.3F, 0.4F}};
  const bf::proto::ServoPacket second{.motor_speed = {0.5F, 0.6F, 0.7F, 0.8F}};
  const std::array<std::byte, 5> junk{};
  ASSERT_TRUE(motor_source.send_to(bf::as_bytes(first), pwm));
  ASSERT_TRUE(motor_source.send_to(std::as_bytes(std::span(junk)), pwm));
  ASSERT_TRUE(motor_source.send_to(bf::as_bytes(second), pwm));

  bf::MotorCommands commands{};
  spin_until([&] {
    link.poll_motors(commands);
    return link.counters().motor_packets == 2 && link.counters().malformed_packets == 1;
  });
  EXPECT_EQ(link.counters().motor_packets, 2U);
  EXPECT_EQ(link.counters().malformed_packets, 1U);
  EXPECT_NEAR(commands.command[3], 0.8, 1e-6);
}
