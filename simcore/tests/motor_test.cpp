#include <gtest/gtest.h>

#include <cmath>

#include <fpvsim/physics/motor.hpp>

#include "test_quad.hpp"

namespace physics = fpvsim::physics;

TEST(MotorTest, StepResponseReaches63PercentAfterOneTimeConstant) {
  const physics::MotorParams params = fpvsim::testing::quad_motor();
  const double dt = 0.001;
  double speed = 0.0;
  for (int i = 0; i < 30; ++i) {
    speed = physics::step_motor(params, speed, 1.0, dt).speed_radps;
  }
  EXPECT_NEAR(speed / params.max_speed_radps, 1.0 - std::exp(-1.0), 1e-9);
}

TEST(MotorTest, SteadyStateThrustIsCoefficientTimesSpeedSquared) {
  const physics::MotorParams params = fpvsim::testing::quad_motor();
  physics::MotorOutput output{};
  double speed = 0.0;
  for (int i = 0; i < 2000; ++i) {
    output = physics::step_motor(params, speed, 0.5, 0.001);
    speed = output.speed_radps;
  }
  const double expected_speed = 0.5 * params.max_speed_radps;
  EXPECT_NEAR(speed, expected_speed, 1e-6);
  EXPECT_NEAR(output.thrust_n, params.thrust_coefficient * expected_speed * expected_speed, 1e-9);
  EXPECT_NEAR(output.reaction_torque_nm,
              params.torque_coefficient * expected_speed * expected_speed, 1e-9);
}

TEST(MotorTest, HoverCommandProducesRequestedThrust) {
  const physics::MotorParams params = fpvsim::testing::quad_motor();
  const double thrust = 1.2;
  const double command = physics::hover_command(params, thrust);
  double speed = 0.0;
  physics::MotorOutput output{};
  for (int i = 0; i < 2000; ++i) {
    output = physics::step_motor(params, speed, command, 0.001);
    speed = output.speed_radps;
  }
  EXPECT_NEAR(output.thrust_n, thrust, 1e-9);
}

TEST(MotorTest, ReactionTorqueIncludesRotorAcceleration) {
  const physics::MotorParams params = fpvsim::testing::quad_motor();
  const physics::MotorOutput output = physics::step_motor(params, 1000.0, 1.0, 0.001);
  const double drag_only = params.torque_coefficient * output.speed_radps * output.speed_radps;
  const double acceleration = (output.speed_radps - 1000.0) / 0.001;
  EXPECT_GT(acceleration, 0.0);
  EXPECT_NEAR(output.reaction_torque_nm, drag_only + params.rotor_inertia_kg_m2 * acceleration,
              1e-12);
}

TEST(MotorTest, CommandIsClampedAndSpeedNeverNegative) {
  const physics::MotorParams params = fpvsim::testing::quad_motor();
  EXPECT_LE(physics::step_motor(params, 2500.0, 2.0, 0.001).speed_radps, params.max_speed_radps);
  EXPECT_GE(physics::step_motor(params, 1.0, -1.0, 10.0).speed_radps, 0.0);
}
