#pragma once

#include <array>
#include <cstddef>

#include <Eigen/Core>

#include <fpvsim/env/atmosphere.hpp>
#include <fpvsim/physics/aero.hpp>
#include <fpvsim/physics/battery.hpp>
#include <fpvsim/physics/contact.hpp>
#include <fpvsim/physics/motor.hpp>
#include <fpvsim/physics/propulsion.hpp>
#include <fpvsim/physics/rigid_body.hpp>
#include <fpvsim/sensors/imu.hpp>
#include <fpvsim/sensors/imu_noise.hpp>
#include <fpvsim/sim/failures.hpp>

namespace fpvsim::sim {

using MotorCommandArray = std::array<double, physics::kMaxMotors>;

struct VehicleParams {
  physics::MassProperties mass;
  std::size_t motor_count;
  std::array<physics::MotorMount, physics::kMaxMotors> mounts;
  std::array<physics::MotorParams, physics::kMaxMotors> motors;
  std::array<physics::PropParams, physics::kMaxMotors> props;
  physics::BatteryParams battery;
  physics::AeroParams aero;
  physics::ContactParams contact;
  double crash_speed_mps;
  Eigen::Vector3d imu_offset_frd;
  sensors::ImuNoiseParams imu_noise;
};

struct VehicleState {
  physics::RigidBodyState body;
  std::array<double, physics::kMaxMotors> motor_speed_radps;
  std::array<double, physics::kMaxMotors> motor_consumed_ah;
  std::array<double, physics::kMaxMotors> motor_bus_current_a;
  std::array<bool, physics::kMaxMotors> motor_desync;
  physics::BatteryState battery;
  bool crashed;
};

struct StepResult {
  physics::Derivatives rates;
  sensors::ImuSample imu;        // with noise, what the flight controller sees
  sensors::ImuSample imu_ideal;  // physically exact, for logs and tests
  sensors::BaroSample baro;
  std::array<physics::MotorOutput, physics::kMaxMotors> motors;
  bool touching;
};

class Vehicle {
 public:
  Vehicle(VehicleParams params, const physics::RigidBodyState& spawn);

  // Ground is the plane NED z = 0; time_s only phases the vibration model
  StepResult step(const MotorCommandArray& commands, const env::Air& air, double time_s,
                  double dt_s);
  // wind_ned is the air's own velocity; drag and rotor inflow see the velocity relative to it
  StepResult step(const MotorCommandArray& commands, const env::Air& air,
                  const Eigen::Vector3d& wind_ned, double time_s, double dt_s);
  void reset(const physics::RigidBodyState& spawn);
  // Hook for failure injection: a desynced ESC stops driving and the rotor freewheels
  void set_motor_desync(std::size_t index, bool on);
  // Failures change only these; healthy_modifiers() is the normal drone
  void set_modifiers(const FailureModifiers& modifiers);
  [[nodiscard]] const FailureModifiers& modifiers() const { return modifiers_; }
  // Swaps the model and resets to spawn; params are copied, no heap involved
  void reload(const VehicleParams& params, const physics::RigidBodyState& spawn);

  [[nodiscard]] const VehicleParams& params() const { return params_; }
  [[nodiscard]] const VehicleState& state() const { return state_; }
  [[nodiscard]] double hover_command() const;

 private:
  VehicleParams params_;
  VehicleState state_;
  sensors::ImuNoise imu_noise_;
  FailureModifiers modifiers_ = healthy_modifiers();
};

// Level, at rest, with the lowest contact point on the ground (NED z = 0) plus height_agl_m
physics::RigidBodyState spawn_state(const VehicleParams& params, double north_m, double east_m,
                                    double height_agl_m, double heading_rad);

}  // namespace fpvsim::sim
