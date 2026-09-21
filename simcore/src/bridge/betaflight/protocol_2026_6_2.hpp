#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

// Native-layout mirrors of the SITL packets in src/platform/SIMULATOR/target/SITL/target.h
namespace fpvsim::bridge::betaflight::v2026_6_2 {

inline constexpr std::size_t kMaxRcChannels = 16;
inline constexpr std::size_t kMaxPwmChannels = 16;
inline constexpr std::size_t kMotorSpeedCount = 4;

inline constexpr std::uint16_t kPortPwmRaw = 9001;
inline constexpr std::uint16_t kPortPwm = 9002;
inline constexpr std::uint16_t kPortFdm = 9003;
inline constexpr std::uint16_t kPortRc = 9004;

struct FdmPacket {
  double timestamp;
  std::array<double, 3> imu_angular_velocity_rpy;
  std::array<double, 3> imu_linear_acceleration_xyz;
  std::array<double, 4> imu_orientation_quat;
  std::array<double, 3> velocity_xyz;
  std::array<double, 3> position_xyz;
  double pressure;
};

struct RcPacket {
  double timestamp;
  std::array<std::uint16_t, kMaxRcChannels> channels;
};

struct ServoPacket {
  std::array<float, kMotorSpeedCount> motor_speed;
};

struct ServoPacketRaw {
  std::uint16_t motor_count;
  std::array<float, kMaxPwmChannels> pwm_output_raw;
};

static_assert(std::is_trivially_copyable_v<FdmPacket> && std::is_standard_layout_v<FdmPacket>);
static_assert(std::is_trivially_copyable_v<RcPacket> && std::is_standard_layout_v<RcPacket>);
static_assert(std::is_trivially_copyable_v<ServoPacket> && std::is_standard_layout_v<ServoPacket>);
static_assert(std::is_trivially_copyable_v<ServoPacketRaw> &&
              std::is_standard_layout_v<ServoPacketRaw>);

static_assert(sizeof(FdmPacket) == 144);
static_assert(offsetof(FdmPacket, imu_angular_velocity_rpy) == 8);
static_assert(offsetof(FdmPacket, imu_linear_acceleration_xyz) == 32);
static_assert(offsetof(FdmPacket, imu_orientation_quat) == 56);
static_assert(offsetof(FdmPacket, velocity_xyz) == 88);
static_assert(offsetof(FdmPacket, position_xyz) == 112);
static_assert(offsetof(FdmPacket, pressure) == 136);

static_assert(sizeof(RcPacket) == 40);
static_assert(offsetof(RcPacket, channels) == 8);

static_assert(sizeof(ServoPacket) == 16);

static_assert(sizeof(ServoPacketRaw) == 68);
static_assert(offsetof(ServoPacketRaw, pwm_output_raw) == 4);

}  // namespace fpvsim::bridge::betaflight::v2026_6_2
