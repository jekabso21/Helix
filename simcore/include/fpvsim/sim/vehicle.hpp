#pragma once

#include <array>
#include <cstddef>

#include <Eigen/Core>

#include <fpvsim/physics/aero.hpp>
#include <fpvsim/physics/contact.hpp>
#include <fpvsim/physics/motor.hpp>
#include <fpvsim/physics/propulsion.hpp>
#include <fpvsim/physics/rigid_body.hpp>
#include <fpvsim/sensors/imu.hpp>

namespace fpvsim::sim {

using MotorCommandArray = std::array<double, physics::kMaxMotors>;

struct VehicleParams {
  physics::MassProperties mass;
  std::size_t motor_count;
  std::array<physics::MotorMount, physics::kMaxMotors> mounts;
  std::array<physics::MotorParams, physics::kMaxMotors> motors;
  physics::AeroParams aero;
  physics::ContactParams contact;
  double crash_speed_mps;
  Eigen::Vector3d imu_offset_frd;
};

struct VehicleState {
  physics::RigidBodyState body;
  std::array<double, physics::kMaxMotors> motor_speed_radps;
  bool crashed;
};

struct StepResult {
  physics::Derivatives rates;
  sensors::ImuSample imu;
  std::array<physics::MotorOutput, physics::kMaxMotors> motors;
  bool touching;
};

class Vehicle {
 public:
  Vehicle(VehicleParams params, const physics::RigidBodyState& spawn);

  StepResult step(const MotorCommandArray& commands, double air_density_kg_m3, double dt_s);
  void reset(const physics::RigidBodyState& spawn);
  // Swaps the model and resets to spawn; params are copied, no heap involved
  void reload(const VehicleParams& params, const physics::RigidBodyState& spawn);

  [[nodiscard]] const VehicleParams& params() const { return params_; }
  [[nodiscard]] const VehicleState& state() const { return state_; }
  [[nodiscard]] double hover_command() const;

 private:
  VehicleParams params_;
  VehicleState state_;
};

// Level, at rest, with the lowest contact point on the ground (NED z = 0) plus height_agl_m
physics::RigidBodyState spawn_state(const VehicleParams& params, double north_m, double east_m,
                                    double height_agl_m, double heading_rad);

}  // namespace fpvsim::sim
