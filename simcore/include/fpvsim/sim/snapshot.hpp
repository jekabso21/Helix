#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include <fpvsim/input/mapping.hpp>
#include <fpvsim/physics/propulsion.hpp>

namespace fpvsim::sim {

// One step's state as seen by the I/O thread; trivially copyable so it can cross the queue
struct Snapshot {
  std::int64_t sim_time_ns;
  std::int64_t step_index;
  std::array<double, 3> position_ned;
  std::array<double, 3> velocity_ned;
  std::array<double, 4> q_ned_from_frd;
  std::array<double, 3> angular_rate_frd;
  std::uint8_t motor_count;
  std::array<double, physics::kMaxMotors> motor_command;
  std::array<double, physics::kMaxMotors> motor_rpm;
  std::array<double, physics::kMaxMotors> motor_thrust_n;
  std::array<std::uint16_t, 16> rc_channels_us;
  std::array<float, input::kMaxAxes> raw_axes;
  std::array<std::uint8_t, input::kMaxButtons> raw_buttons;
  std::uint8_t raw_axis_count;
  std::uint8_t raw_button_count;
  bool device_connected;
  bool armed;
  bool crashed;
  bool touching;
  bool paused;
  double air_density_kg_m3;
  std::uint64_t overruns;
  std::uint64_t motor_packets;
  std::uint64_t malformed_packets;
  double step_time_mean_us;
  double step_time_p99_us;
  double step_time_max_us;
};

enum class CommandType : std::uint8_t {
  kReset,
  kPause,
  kResume,
  kShutdown,
  kSetInputMapping,
  kSelectInputDevice,
  kReloadModel
};

struct Command {
  std::uint32_t client;
  std::int64_t request_id;
  CommandType type;
};

struct CommandResult {
  std::uint32_t client;
  std::int64_t request_id;
  std::int64_t applied_at_ns;
};

}  // namespace fpvsim::sim
