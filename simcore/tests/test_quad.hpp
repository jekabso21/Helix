#pragma once

#include <array>
#include <cstddef>

#include <fpvsim/physics/contact.hpp>
#include <fpvsim/physics/motor.hpp>
#include <fpvsim/physics/propulsion.hpp>
#include <fpvsim/physics/rigid_body.hpp>

namespace fpvsim::testing {

inline constexpr double kArmM = 0.08;
inline constexpr double kMassKg = 0.5;
inline constexpr std::size_t kMotorCount = 4;

inline physics::MassProperties quad_mass() {
  return physics::make_mass_properties(kMassKg, Eigen::Vector3d(0.003, 0.003, 0.005).asDiagonal());
}

inline physics::MotorParams quad_motor() {
  return physics::MotorParams{.max_speed_radps = 2500.0,
                              .time_constant_s = 0.03,
                              .thrust_coefficient = 1.5e-6,
                              .torque_coefficient = 2.0e-8,
                              .rotor_inertia_kg_m2 = 6.0e-6,
                              .rotor_drag_coefficient = 6.0e-5};
}

// Betaflight quad X: 1 rear right CW, 2 front right CCW, 3 rear left CCW, 4 front left CW
inline std::array<physics::MotorMount, physics::kMaxMotors> quad_mounts() {
  const Eigen::Vector3d up(0.0, 0.0, -1.0);
  std::array<physics::MotorMount, physics::kMaxMotors> mounts{};
  mounts[0] = {.position_frd = {-kArmM, kArmM, 0.0}, .axis_frd = up, .spin = 1.0};
  mounts[1] = {.position_frd = {kArmM, kArmM, 0.0}, .axis_frd = up, .spin = -1.0};
  mounts[2] = {.position_frd = {-kArmM, -kArmM, 0.0}, .axis_frd = up, .spin = -1.0};
  mounts[3] = {.position_frd = {kArmM, -kArmM, 0.0}, .axis_frd = up, .spin = 1.0};
  return mounts;
}

inline std::array<physics::MotorParams, physics::kMaxMotors> quad_motors() {
  std::array<physics::MotorParams, physics::kMaxMotors> motors{};
  motors.fill(quad_motor());
  return motors;
}

inline physics::ContactParams quad_contact() {
  physics::ContactParams params{};
  params.points_frd[0] = {kArmM, kArmM, 0.02};
  params.points_frd[1] = {kArmM, -kArmM, 0.02};
  params.points_frd[2] = {-kArmM, kArmM, 0.02};
  params.points_frd[3] = {-kArmM, -kArmM, 0.02};
  params.point_count = 4;
  params.stiffness_n_per_m = 3000.0;
  params.damping_n_s_per_m = 30.0;
  params.friction = 0.6;
  params.friction_regularization_mps = 0.01;
  return params;
}

inline physics::Loads no_loads() {
  return physics::Loads{.force_frd = Eigen::Vector3d::Zero(),
                        .force_ned = Eigen::Vector3d::Zero(),
                        .torque_frd = Eigen::Vector3d::Zero(),
                        .rotor_momentum_frd = Eigen::Vector3d::Zero()};
}

}  // namespace fpvsim::testing
