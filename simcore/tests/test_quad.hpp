#pragma once

#include <array>
#include <cstddef>

#include <fpvsim/physics/battery.hpp>
#include <fpvsim/physics/contact.hpp>
#include <fpvsim/physics/motor.hpp>
#include <fpvsim/physics/propulsion.hpp>
#include <fpvsim/physics/rigid_body.hpp>
#include <fpvsim/sensors/imu_noise.hpp>

namespace fpvsim::testing {

inline constexpr double kArmM = 0.08;
inline constexpr double kMassKg = 0.5;
inline constexpr std::size_t kMotorCount = 4;

inline physics::MassProperties quad_mass() {
  return physics::make_mass_properties(kMassKg, Eigen::Vector3d(0.003, 0.003, 0.005).asDiagonal());
}

inline constexpr double kBusVoltage = 24.0;

inline physics::MotorParams quad_motor() {
  return physics::MotorParams{.model = physics::MotorModel::kFirstOrder,
                              .max_speed_radps = 2500.0,
                              .time_constant_s = 0.03,
                              .reference_voltage_v = kBusVoltage,
                              .kv_radps_per_v = 0.0,
                              .resistance_ohm = 0.0,
                              .no_load_current_a = 0.0,
                              .brake_current_a = 0.0,
                              .rotor_inertia_kg_m2 = 6.0e-6,
                              .pole_pairs = 7};
}

// The placeholder 2207 1900 kV: DC-equivalent 0.18 ohm, no-load speed at 24 V is 4770 rad/s
inline physics::MotorParams dc_motor() {
  return physics::MotorParams{.model = physics::MotorModel::kDc,
                              .max_speed_radps = 0.0,
                              .time_constant_s = 0.0,
                              .reference_voltage_v = kBusVoltage,
                              .kv_radps_per_v = 1900.0 * 2.0 * 3.14159265358979 / 60.0,
                              .resistance_ohm = 0.18,
                              .no_load_current_a = 1.5,
                              .brake_current_a = 10.0,
                              .rotor_inertia_kg_m2 = 3.0e-6,
                              .pole_pairs = 7};
}

inline physics::PropParams quad_prop() {
  return physics::PropParams{.thrust_coefficient = 1.5e-6,
                             .torque_coefficient = 2.0e-8,
                             .reference_density_kg_m3 = 1.225,
                             .radius_m = 0.0635,
                             .pitch_m = 0.109,
                             .inflow_coefficient = 0.0,
                             .rotor_drag_coefficient = 6.0e-5,
                             .blades = 3};
}

inline std::array<physics::PropParams, physics::kMaxMotors> quad_props() {
  std::array<physics::PropParams, physics::kMaxMotors> props{};
  props.fill(quad_prop());
  return props;
}

inline physics::MotorInput still_air(double command) {
  return physics::MotorInput{.command = command,
                             .bus_voltage_v = kBusVoltage,
                             .air_density_kg_m3 = 1.225,
                             .axial_inflow_mps = 0.0,
                             .height_above_ground_m = 100.0};
}

inline physics::BatteryParams quad_battery() {
  return physics::BatteryParams{
      .cells = 6,
      .capacity_ah = 1.1,
      .cell_resistance_ohm = 0.008,
      .connector_resistance_ohm = 0.002,
      .avionics_current_a = 0.8,
      .esc_cutoff_v = 15.0,
      .initial_soc = 1.0,
      .rc_resistance_ohm = 0.0,
      .rc_capacitance_f = 0.0,
      .ocv_v = {3.30, 3.60, 3.70, 3.75, 3.79, 3.83, 3.87, 3.93, 4.00, 4.10, 4.20}};
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

inline sensors::ImuNoiseParams quiet_imu() {
  return sensors::ImuNoiseParams{.sample_rate_hz = 1000.0,
                                 .gyro_noise_density = 0.0,
                                 .gyro_bias_walk = 0.0,
                                 .gyro_range_radps = 0.0,
                                 .accel_noise_density = 0.0,
                                 .accel_bias_walk = 0.0,
                                 .accel_range_mps2 = 0.0,
                                 .vibration_imbalance = 0.0,
                                 .vibration_harmonic2 = 0.0,
                                 .vibration_blade_pass = 0.0,
                                 .vibration_gyro_gain = 0.0,
                                 .baro_noise_pa = 0.0,
                                 .baro_bias_pa = 0.0,
                                 .seed = 1};
}

inline physics::Loads no_loads() {
  return physics::Loads{.force_frd = Eigen::Vector3d::Zero(),
                        .force_ned = Eigen::Vector3d::Zero(),
                        .torque_frd = Eigen::Vector3d::Zero(),
                        .rotor_momentum_frd = Eigen::Vector3d::Zero()};
}

}  // namespace fpvsim::testing
