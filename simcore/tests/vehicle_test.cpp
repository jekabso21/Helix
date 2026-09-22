#include <gtest/gtest.h>

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
                            .aero = {.drag_area_frd_m2 = Vector3d(0.01, 0.01, 0.02),
                                     .center_of_pressure_frd = Vector3d::Zero(),
                                     .angular_damping = Vector3d(0.0005, 0.0005, 0.0005)},
                            .contact = quad::quad_contact(),
                            .crash_speed_mps = 6.0,
                            .imu_offset_frd = Vector3d::Zero()};
}

constexpr double kRho = 1.225;

}  // namespace

TEST(VehicleTest, SpawnRestsOnTheGroundWithoutSinkingOrBouncing) {
  const sim::VehicleParams params = quad_params();
  sim::Vehicle vehicle(params, sim::spawn_state(params, 0.0, 0.0, 0.0, 0.0));
  const double start_z = vehicle.state().body.position_ned.z();
  for (int i = 0; i < 3000; ++i) {
    vehicle.step({}, kRho, 0.001);
  }
  EXPECT_NEAR(vehicle.state().body.position_ned.z(), start_z, 0.002);
  EXPECT_FALSE(vehicle.state().crashed);
  EXPECT_LT(vehicle.state().body.velocity_ned.norm(), 1e-4);
}

TEST(VehicleTest, HoverCommandReachesEquilibriumInStillAir) {
  const sim::VehicleParams params = quad_params();
  sim::Vehicle vehicle(params, sim::spawn_state(params, 0.0, 0.0, 10.0, 0.0));
  sim::MotorCommandArray commands{};
  commands.fill(vehicle.hover_command());
  const double start_z = vehicle.state().body.position_ned.z();
  sim::StepResult result{};
  for (int i = 0; i < 5000; ++i) {
    result = vehicle.step(commands, kRho, 0.001);
  }
  // Motors spin up from rest, so the drone sinks a little before thrust balances weight
  EXPECT_GT(vehicle.state().body.position_ned.z(), start_z);
  EXPECT_LT(vehicle.state().body.position_ned.z(), start_z + 3.0);
  // Only the drag on the residual sink velocity remains, about 0.005 m/s^2
  EXPECT_NEAR(result.rates.acceleration_ned.z(), 0.0, 1e-2);
  EXPECT_NEAR(result.imu.specific_force_frd.z(), -fpvsim::kStandardGravityMps2, 1e-2);
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
    vehicle.step({}, kRho, 0.001);
    crashed = vehicle.state().crashed;
  }
  ASSERT_TRUE(crashed);
  for (int i = 0; i < 1000; ++i) {
    vehicle.step(full, kRho, 0.001);
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
