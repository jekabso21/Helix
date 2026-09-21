#include <gtest/gtest.h>

#include <cstddef>
#include <cstring>

#include "bridge/betaflight/protocol_2026_6_2.hpp"
#include "upstream_sitl_packets.hpp"

namespace bf = fpvsim::bridge::betaflight::v2026_6_2;

static_assert(SIMULATOR_MAX_RC_CHANNELS == bf::kMaxRcChannels);
static_assert(SIMULATOR_MAX_PWM_CHANNELS == bf::kMaxPwmChannels);

static_assert(sizeof(upstream::fdm_packet) == sizeof(bf::FdmPacket));
static_assert(offsetof(upstream::fdm_packet, timestamp) == offsetof(bf::FdmPacket, timestamp));
static_assert(offsetof(upstream::fdm_packet, imu_angular_velocity_rpy) ==
              offsetof(bf::FdmPacket, imu_angular_velocity_rpy));
static_assert(offsetof(upstream::fdm_packet, imu_linear_acceleration_xyz) ==
              offsetof(bf::FdmPacket, imu_linear_acceleration_xyz));
static_assert(offsetof(upstream::fdm_packet, imu_orientation_quat) ==
              offsetof(bf::FdmPacket, imu_orientation_quat));
static_assert(offsetof(upstream::fdm_packet, velocity_xyz) ==
              offsetof(bf::FdmPacket, velocity_xyz));
static_assert(offsetof(upstream::fdm_packet, position_xyz) ==
              offsetof(bf::FdmPacket, position_xyz));
static_assert(offsetof(upstream::fdm_packet, pressure) == offsetof(bf::FdmPacket, pressure));

static_assert(sizeof(upstream::rc_packet) == sizeof(bf::RcPacket));
static_assert(offsetof(upstream::rc_packet, channels) == offsetof(bf::RcPacket, channels));

static_assert(sizeof(upstream::servo_packet) == sizeof(bf::ServoPacket));
static_assert(sizeof(upstream::servo_packet::motor_speed) == sizeof(bf::ServoPacket::motor_speed));

static_assert(sizeof(upstream::servo_packet_raw) == sizeof(bf::ServoPacketRaw));
static_assert(offsetof(upstream::servo_packet_raw, pwm_output_raw) ==
              offsetof(bf::ServoPacketRaw, pwm_output_raw));

TEST(ProtocolLayoutTest, FdmPacketBytesReadBackThroughTheUpstreamStruct) {
  bf::FdmPacket ours{};
  ours.timestamp = 1.5;
  ours.imu_angular_velocity_rpy = {0.1, 0.2, 0.3};
  ours.imu_linear_acceleration_xyz = {-1.0, 2.0, -9.80665};
  ours.imu_orientation_quat = {0.5, -0.5, 0.25, -0.25};
  ours.velocity_xyz = {4.0, 5.0, 6.0};
  ours.position_xyz = {24.0, 56.0, 12.5};
  ours.pressure = 101325.0;

  upstream::fdm_packet theirs{};
  std::memcpy(&theirs, &ours, sizeof(ours));

  EXPECT_EQ(theirs.timestamp, 1.5);
  EXPECT_EQ(theirs.imu_angular_velocity_rpy[0], 0.1);
  EXPECT_EQ(theirs.imu_angular_velocity_rpy[2], 0.3);
  EXPECT_EQ(theirs.imu_linear_acceleration_xyz[0], -1.0);
  EXPECT_EQ(theirs.imu_linear_acceleration_xyz[2], -9.80665);
  EXPECT_EQ(theirs.imu_orientation_quat[0], 0.5);
  EXPECT_EQ(theirs.imu_orientation_quat[3], -0.25);
  EXPECT_EQ(theirs.velocity_xyz[1], 5.0);
  EXPECT_EQ(theirs.position_xyz[0], 24.0);
  EXPECT_EQ(theirs.position_xyz[2], 12.5);
  EXPECT_EQ(theirs.pressure, 101325.0);
}

TEST(ProtocolLayoutTest, UpstreamServoPacketsReadBackThroughOurStructs) {
  upstream::servo_packet_raw theirs_raw{};
  theirs_raw.motorCount = 4;
  theirs_raw.pwm_output_raw[0] = 1100.0F;
  theirs_raw.pwm_output_raw[15] = 1900.0F;
  bf::ServoPacketRaw ours_raw{};
  std::memcpy(&ours_raw, &theirs_raw, sizeof(ours_raw));
  EXPECT_EQ(ours_raw.motor_count, 4);
  EXPECT_EQ(ours_raw.pwm_output_raw[0], 1100.0F);
  EXPECT_EQ(ours_raw.pwm_output_raw[15], 1900.0F);

  upstream::servo_packet theirs{};
  theirs.motor_speed[0] = 0.25F;
  theirs.motor_speed[3] = 1.0F;
  bf::ServoPacket ours{};
  std::memcpy(&ours, &theirs, sizeof(ours));
  EXPECT_EQ(ours.motor_speed[0], 0.25F);
  EXPECT_EQ(ours.motor_speed[3], 1.0F);
}
