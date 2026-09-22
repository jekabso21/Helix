#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

#include <fpvsim/constants.hpp>
#include <fpvsim/sim/vehicle.hpp>

#include "test_quad.hpp"

namespace sim = fpvsim::sim;
namespace quad = fpvsim::testing;
using Eigen::Vector3d;

namespace {

sim::VehicleParams quad_params() {
  return sim::VehicleParams{.mass = quad::quad_mass(),
                            .motor_count = quad::kMotorCount,
                            .mounts = quad::quad_mounts(),
                            .motors = quad::quad_motors(),
                            .props = quad::quad_props(),
                            .battery = quad::quad_battery(),
                            .aero = {.drag_area_frd_m2 = Vector3d(0.01, 0.01, 0.02),
                                     .center_of_pressure_frd = Vector3d::Zero(),
                                     .angular_damping = Vector3d(0.0005, 0.0005, 0.0005)},
                            .contact = quad::quad_contact(),
                            .crash_speed_mps = 6.0,
                            .imu_offset_frd = Vector3d::Zero(),
                            .imu_noise = quad::quiet_imu()};
}

const fpvsim::env::Air kAir{
    .temperature_k = 288.15, .pressure_pa = 101325.0, .density_kg_m3 = 1.225};

}  // namespace

TEST(VehicleTest, SpawnRestsOnTheGroundWithoutSinkingOrBouncing) {
  const sim::VehicleParams params = quad_params();
  sim::Vehicle vehicle(params, sim::spawn_state(params, 0.0, 0.0, 0.0, 0.0));
  const double start_z = vehicle.state().body.position_ned.z();
  for (int i = 0; i < 3000; ++i) {
    vehicle.step({}, kAir, 0.0, 0.001);
  }
  EXPECT_NEAR(vehicle.state().body.position_ned.z(), start_z, 0.002);
  EXPECT_FALSE(vehicle.state().crashed);
  EXPECT_LT(vehicle.state().body.velocity_ned.norm(), 1e-4);
}

TEST(VehicleTest, HoverCommandReachesEquilibriumInStillAir) {
  sim::VehicleParams params = quad_params();
  // hover_command() assumes the open-circuit voltage; no sag keeps the balance exact
  params.battery.cell_resistance_ohm = 0.0;
  params.battery.connector_resistance_ohm = 0.0;
  sim::Vehicle vehicle(params, sim::spawn_state(params, 0.0, 0.0, 10.0, 0.0));
  sim::MotorCommandArray commands{};
  commands.fill(vehicle.hover_command());
  const double start_z = vehicle.state().body.position_ned.z();
  sim::StepResult result{};
  for (int i = 0; i < 5000; ++i) {
    result = vehicle.step(commands, kAir, 0.0, 0.001);
  }
  // Motors spin up from rest, so the drone sinks a little before thrust balances weight
  EXPECT_GT(vehicle.state().body.position_ned.z(), start_z);
  EXPECT_LT(vehicle.state().body.position_ned.z(), start_z + 3.0);
  // Left over: drag on the residual sink velocity and the DShot step of the command (0.1 %)
  EXPECT_NEAR(result.rates.acceleration_ned.z(), 0.0, 3e-2);
  EXPECT_NEAR(result.imu.specific_force_frd.z(), -fpvsim::kStandardGravityMps2, 3e-2);
  EXPECT_LT(vehicle.state().body.angular_rate_frd.norm(), 1e-6);
  EXPECT_FALSE(result.touching);
}

TEST(VehicleTest, FallingFromHeightCrashesAndCutsMotors) {
  const sim::VehicleParams params = quad_params();
  sim::Vehicle vehicle(params, sim::spawn_state(params, 0.0, 0.0, 5.0, 0.0));
  sim::MotorCommandArray full{};
  full.fill(1.0);
  bool crashed = false;
  for (int i = 0; i < 3000 && !crashed; ++i) {
    vehicle.step({}, kAir, 0.0, 0.001);
    crashed = vehicle.state().crashed;
  }
  ASSERT_TRUE(crashed);
  for (int i = 0; i < 1000; ++i) {
    vehicle.step(full, kAir, 0.0, 0.001);
  }
  EXPECT_EQ(vehicle.state().motor_speed_radps[0], 0.0);
  vehicle.reset(sim::spawn_state(params, 0.0, 0.0, 0.0, 0.0));
  EXPECT_FALSE(vehicle.state().crashed);
}

TEST(VehicleTest, HeadingIsAppliedAtSpawn) {
  const sim::VehicleParams params = quad_params();
  const auto state = sim::spawn_state(params, 1.0, 2.0, 0.0, std::numbers::pi / 2.0);
  const Vector3d nose = state.q_ned_from_frd * Vector3d::UnitX();
  EXPECT_NEAR(nose.y(), 1.0, 1e-9);
  EXPECT_DOUBLE_EQ(state.position_ned.x(), 1.0);
  EXPECT_DOUBLE_EQ(state.position_ned.y(), 2.0);
}

TEST(VehicleTest, ReloadSwapsTheModelAndResetsToSpawn) {
  const sim::VehicleParams params = quad_params();
  sim::Vehicle vehicle(params, sim::spawn_state(params, 0.0, 0.0, 5.0, 0.0));
  sim::MotorCommandArray full{};
  full.fill(1.0);
  for (int i = 0; i < 200; ++i) {
    vehicle.step(full, kAir, 0.0, 0.001);
  }
  sim::VehicleParams heavier = params;
  heavier.mass =
      fpvsim::physics::make_mass_properties(2.0 * quad::kMassKg, params.mass.inertia_frd);
  const auto spawn = sim::spawn_state(heavier, 1.0, 2.0, 0.0, 0.0);
  vehicle.reload(heavier, spawn);
  EXPECT_DOUBLE_EQ(vehicle.params().mass.mass_kg, 2.0 * quad::kMassKg);
  EXPECT_NEAR(vehicle.state().body.position_ned.x(), 1.0, 1e-12);
  EXPECT_DOUBLE_EQ(vehicle.state().motor_speed_radps[0], 0.0);
  EXPECT_NEAR(vehicle.hover_command(), std::sqrt(2.0) * sim::Vehicle(params, spawn).hover_command(),
              1e-9);
}

// A CG ahead of the motor centre with equal thrust everywhere pitches the nose up (FRD -y torque)
TEST(VehicleTest, EqualThrustWithAForwardCgPitchesNoseUpByTheMomentBalance) {
  sim::VehicleParams params = quad_params();
  const double shift = 0.010;  // CG 10 mm ahead: every motor moves 10 mm aft relative to it
  for (std::size_t i = 0; i < quad::kMotorCount; ++i) {
    params.mounts[i].position_frd.x() -= shift;
  }
  sim::Vehicle vehicle(params, sim::spawn_state(params, 0.0, 0.0, 5.0, 0.0));
  sim::MotorCommandArray half{};
  half.fill(0.5);
  // first step: the body has no angular rate yet, so no damping or gyroscopic terms
  const sim::StepResult result = vehicle.step(half, kAir, 0.0, 0.001);
  double total_thrust = 0.0;
  for (std::size_t i = 0; i < quad::kMotorCount; ++i) {
    total_thrust += result.motors[i].thrust_n;
  }
  // moment about y of the four upward thrusts at x = -shift: sum(r x F)_y = -shift * T_total
  const double expected_pitch_accel = -shift * total_thrust / params.mass.inertia_frd(1, 1);
  EXPECT_LT(expected_pitch_accel, 0.0);
  EXPECT_NEAR(result.rates.angular_acceleration_frd.y(), expected_pitch_accel,
              1e-6 * std::abs(expected_pitch_accel));
}
