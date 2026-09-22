#include <gtest/gtest.h>

#include <array>

#include <fpvsim/constants.hpp>
#include <fpvsim/physics/propulsion.hpp>

#include "test_quad.hpp"

namespace physics = fpvsim::physics;
namespace quad = fpvsim::testing;

namespace {

std::array<physics::MotorOutput, physics::kMaxMotors> steady_outputs(
    const std::array<double, quad::kMotorCount>& speeds) {
  const physics::PropParams params = quad::quad_prop();
  std::array<physics::MotorOutput, physics::kMaxMotors> outputs{};
  for (std::size_t i = 0; i < quad::kMotorCount; ++i) {
    outputs[i] = physics::MotorOutput{
        .speed_radps = speeds[i],
        .thrust_n = params.thrust_coefficient * speeds[i] * speeds[i],
        .reaction_torque_nm = params.torque_coefficient * speeds[i] * speeds[i],
        .current_a = 0.0,
        .bus_current_a = 0.0};
  }
  return outputs;
}

int sign(double value) {
  if (value > 0.0) {
    return 1;
  }
  return value < 0.0 ? -1 : 0;
}

}  // namespace

TEST(PropulsionTest, HoverThrustEqualsWeight) {
  const physics::MotorParams motor = quad::quad_motor();
  const double weight = quad::kMassKg * fpvsim::kStandardGravityMps2;
  const double speed =
      physics::hover_command(motor, quad::quad_prop(), weight / 4.0, quad::kBusVoltage) *
      motor.max_speed_radps;
  const physics::Loads loads =
      physics::propulsion_loads(quad::quad_mounts(), quad::quad_motors(),
                                steady_outputs({speed, speed, speed, speed}), quad::kMotorCount);
  const physics::Derivatives rates = physics::derivatives(
      physics::level_state_at(Eigen::Vector3d::Zero()), quad::quad_mass(), loads);
  EXPECT_LT(rates.acceleration_ned.norm(), 1e-9);
  EXPECT_LT(rates.angular_acceleration_frd.norm(), 1e-9);
  EXPECT_LT(loads.rotor_momentum_frd.norm(), 1e-12);
}

TEST(PropulsionTest, PerMotorTorqueSignsMatchBetaflightOrderAndSpin) {
  // Expected torque signs (roll, pitch, yaw) when only that motor runs faster
  const std::array<std::array<int, 3>, quad::kMotorCount> expected = {{
      {-1, -1, -1},  // 1 rear right, clockwise
      {-1, +1, +1},  // 2 front right, counter-clockwise
      {+1, -1, +1},  // 3 rear left, counter-clockwise
      {+1, +1, -1},  // 4 front left, clockwise
  }};
  for (std::size_t motor = 0; motor < quad::kMotorCount; ++motor) {
    std::array<double, quad::kMotorCount> speeds = {1000.0, 1000.0, 1000.0, 1000.0};
    speeds[motor] = 1500.0;
    const physics::Loads loads = physics::propulsion_loads(
        quad::quad_mounts(), quad::quad_motors(), steady_outputs(speeds), quad::kMotorCount);
    for (int axis = 0; axis < 3; ++axis) {
      EXPECT_EQ(sign(loads.torque_frd[axis]), expected[motor][static_cast<std::size_t>(axis)])
          << "motor " << motor + 1 << " axis " << axis;
    }
  }
}

TEST(PropulsionTest, BetaflightCorrectionsOpposeTheDisturbance) {
  // Motors Betaflight speeds up against a true disturbance, measured on the firmware
  const std::array<std::array<std::size_t, 2>, 3> sped_up = {{{0, 1}, {0, 2}, {0, 3}}};
  for (std::size_t axis = 0; axis < 3; ++axis) {
    std::array<double, quad::kMotorCount> speeds = {1000.0, 1000.0, 1000.0, 1000.0};
    speeds[sped_up[axis][0]] = 1300.0;
    speeds[sped_up[axis][1]] = 1300.0;
    const physics::Loads loads = physics::propulsion_loads(
        quad::quad_mounts(), quad::quad_motors(), steady_outputs(speeds), quad::kMotorCount);
    EXPECT_LT(loads.torque_frd[static_cast<Eigen::Index>(axis)], 0.0) << "axis " << axis;
  }
}

TEST(PropulsionTest, RotorDragOpposesInPlaneAirspeedOnly) {
  const auto outputs = steady_outputs({1000.0, 1000.0, 1000.0, 1000.0});
  const physics::Loads sideways =
      physics::rotor_drag_loads(quad::quad_mounts(), quad::quad_props(), outputs, quad::kMotorCount,
                                Eigen::Vector3d(10.0, 0.0, 0.0), Eigen::Vector3d::Zero());
  EXPECT_NEAR(sideways.force_frd.x(), -4.0 * 6.0e-5 * 1000.0 * 10.0, 1e-12);
  EXPECT_NEAR(sideways.force_frd.y(), 0.0, 1e-12);
  EXPECT_NEAR(sideways.force_frd.z(), 0.0, 1e-12);
  EXPECT_LT(sideways.torque_frd.norm(), 1e-12);
  const physics::Loads axial =
      physics::rotor_drag_loads(quad::quad_mounts(), quad::quad_props(), outputs, quad::kMotorCount,
                                Eigen::Vector3d(0.0, 0.0, -10.0), Eigen::Vector3d::Zero());
  EXPECT_LT(axial.force_frd.norm(), 1e-12);
}

TEST(PropulsionTest, RotorDragDampsBodyRates) {
  const auto outputs = steady_outputs({1000.0, 1000.0, 1000.0, 1000.0});
  const physics::Loads spinning =
      physics::rotor_drag_loads(quad::quad_mounts(), quad::quad_props(), outputs, quad::kMotorCount,
                                Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, 5.0));
  EXPECT_LT(spinning.torque_frd.z(), 0.0);
  EXPECT_LT(spinning.force_frd.norm(), 1e-12);
}

TEST(PropulsionTest, ClockwiseRotorMomentumPointsDown) {
  const physics::Loads loads =
      physics::propulsion_loads(quad::quad_mounts(), quad::quad_motors(),
                                steady_outputs({1000.0, 0.0, 0.0, 0.0}), quad::kMotorCount);
  EXPECT_NEAR(loads.rotor_momentum_frd.z(), 6.0e-6 * 1000.0, 1e-15);
}
