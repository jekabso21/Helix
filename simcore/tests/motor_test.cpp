#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

#include <fpvsim/physics/motor.hpp>

#include "test_quad.hpp"

namespace physics = fpvsim::physics;
namespace quad = fpvsim::testing;

namespace {

physics::MotorOutput settle(const physics::MotorParams& motor, const physics::PropParams& prop,
                            const physics::MotorInput& input, int steps = 3000) {
  physics::MotorOutput output{};
  double speed = 0.0;
  for (int i = 0; i < steps; ++i) {
    output = physics::step_motor(motor, prop, speed, input, 0.001);
    speed = output.speed_radps;
  }
  return output;
}

}  // namespace

TEST(MotorTest, CommandIsQuantizedToDshotSteps) {
  EXPECT_DOUBLE_EQ(physics::quantize_command(1.0), 1.0);
  EXPECT_DOUBLE_EQ(physics::quantize_command(0.0), 0.0);
  EXPECT_DOUBLE_EQ(physics::quantize_command(1.0 / 1999.0 * 0.4), 0.0);
  EXPECT_DOUBLE_EQ(physics::quantize_command(1.0 / 1999.0 * 0.6), 1.0 / 1999.0);
  EXPECT_DOUBLE_EQ(physics::quantize_command(7.0), 1.0);
}

TEST(MotorTest, FirstOrderStepResponseReaches63PercentAfterOneTimeConstant) {
  const physics::MotorParams motor = quad::quad_motor();
  double speed = 0.0;
  for (int i = 0; i < 30; ++i) {
    speed = physics::step_motor(motor, quad::quad_prop(), speed, quad::still_air(1.0), 0.001)
                .speed_radps;
  }
  EXPECT_NEAR(speed / motor.max_speed_radps, 1.0 - std::exp(-1.0), 1e-9);
}

TEST(MotorTest, FirstOrderSteadyStateScalesWithVoltageAndThrustWithSpeedSquared) {
  const physics::MotorParams motor = quad::quad_motor();
  const physics::PropParams prop = quad::quad_prop();
  const double command = physics::quantize_command(0.5);
  const physics::MotorOutput output = settle(motor, prop, quad::still_air(0.5));
  const double expected_speed = command * motor.max_speed_radps;
  EXPECT_NEAR(output.speed_radps, expected_speed, 1e-6);
  // ground effect at the test height of 100 m is 1 + 2.5e-8
  EXPECT_NEAR(output.thrust_n, prop.thrust_coefficient * expected_speed * expected_speed, 1e-6);
  EXPECT_NEAR(output.reaction_torque_nm, prop.torque_coefficient * expected_speed * expected_speed,
              1e-9);
  physics::MotorInput sagged = quad::still_air(0.5);
  sagged.bus_voltage_v = 0.5 * quad::kBusVoltage;
  EXPECT_NEAR(settle(motor, prop, sagged).speed_radps, 0.5 * expected_speed, 1e-6);
}

TEST(MotorTest, DcSteadyStateMatchesKvTimesVoltageMinusIrDrop) {
  const physics::MotorParams motor = quad::dc_motor();
  const physics::PropParams prop = quad::quad_prop();
  const physics::MotorOutput output = settle(motor, prop, quad::still_air(0.7));
  const double kt = 1.0 / motor.kv_radps_per_v;
  const double v_m = physics::quantize_command(0.7) * quad::kBusVoltage;
  // torque balance: Kt (I - I0) = k_q w^2, electrical: I = (V - Ke w) / R
  const double w = output.speed_radps;
  const double current = (v_m - kt * w) / motor.resistance_ohm;
  EXPECT_NEAR(kt * (current - motor.no_load_current_a), prop.torque_coefficient * w * w,
              1e-6 * prop.torque_coefficient * w * w);
  EXPECT_NEAR(w, motor.kv_radps_per_v * (v_m - current * motor.resistance_ohm), 1e-6);
  EXPECT_NEAR(output.current_a, current, 1e-9);
  EXPECT_NEAR(output.bus_current_a, physics::quantize_command(0.7) * current, 1e-9);
  EXPECT_GT(output.current_a, motor.no_load_current_a);
}

TEST(MotorTest, DcTimeConstantIsRotorInertiaTimesResistanceOverKtKe) {
  physics::MotorParams motor = quad::dc_motor();
  motor.no_load_current_a = 0.0;
  physics::PropParams prop = quad::quad_prop();
  prop.torque_coefficient = 0.0;  // pure electromechanical response
  const double kt = 1.0 / motor.kv_radps_per_v;
  const double tau = motor.rotor_inertia_kg_m2 * motor.resistance_ohm / (kt * kt);
  const double dt = 1e-5;
  double speed = 0.0;
  const int steps = static_cast<int>(std::lround(tau / dt));
  for (int i = 0; i < steps; ++i) {
    speed = physics::step_motor(motor, prop, speed, quad::still_air(1.0), dt).speed_radps;
  }
  const double final_speed = motor.kv_radps_per_v * quad::kBusVoltage;
  EXPECT_NEAR(speed / final_speed, 1.0 - std::exp(-1.0), 2e-3);
}

TEST(MotorTest, ThrustScalesWithAirDensity) {
  const physics::PropParams prop = quad::quad_prop();
  physics::MotorInput thin = quad::still_air(0.6);
  thin.air_density_kg_m3 = 0.5 * prop.reference_density_kg_m3;
  const physics::MotorOutput sea = settle(quad::quad_motor(), prop, quad::still_air(0.6));
  const physics::MotorOutput high = settle(quad::quad_motor(), prop, thin);
  EXPECT_NEAR(high.speed_radps, sea.speed_radps, 1e-9);
  EXPECT_NEAR(high.thrust_n, 0.5 * sea.thrust_n, 1e-9);
  EXPECT_NEAR(high.reaction_torque_nm, 0.5 * sea.reaction_torque_nm, 1e-9);
}

TEST(MotorTest, InflowLossReducesThrustMonotonicallyAndIsClamped) {
  physics::PropParams prop = quad::quad_prop();
  prop.inflow_coefficient = 1.0;
  const double w = 2000.0;
  const double pitch_speed = prop.pitch_m * w / (2.0 * std::numbers::pi);
  EXPECT_DOUBLE_EQ(physics::inflow_efficiency(prop, w, 0.0), 1.0);
  EXPECT_DOUBLE_EQ(physics::inflow_efficiency(prop, w, -5.0), 1.0);  // descending: no loss
  double previous = 1.0;
  for (double v = 1.0; v < pitch_speed; v += 2.0) {
    const double eta = physics::inflow_efficiency(prop, w, v);
    EXPECT_LT(eta, previous);
    EXPECT_NEAR(eta, 1.0 - v / pitch_speed, 1e-12);
    previous = eta;
  }
  EXPECT_DOUBLE_EQ(physics::inflow_efficiency(prop, w, 2.0 * pitch_speed), 0.0);
  prop.inflow_coefficient = 0.0;
  EXPECT_DOUBLE_EQ(physics::inflow_efficiency(prop, w, 10.0), 1.0);
}

TEST(MotorTest, GroundEffectRaisesThrustNearTheGroundUpToFourThirds) {
  const physics::PropParams prop = quad::quad_prop();
  EXPECT_NEAR(physics::ground_effect_factor(prop, 100.0), 1.0, 1e-6);
  EXPECT_NEAR(physics::ground_effect_factor(prop, prop.radius_m), 1.0 / (1.0 - 1.0 / 16.0), 1e-12);
  EXPECT_NEAR(physics::ground_effect_factor(prop, 0.0), 4.0 / 3.0, 1e-12);
  EXPECT_NEAR(physics::ground_effect_factor(prop, prop.radius_m / 2.0), 4.0 / 3.0, 1e-12);
}

TEST(MotorTest, HoverCommandProducesRequestedThrustForBothModels) {
  const physics::PropParams prop = quad::quad_prop();
  const double thrust = 1.2;
  for (const physics::MotorParams& motor : {quad::quad_motor(), quad::dc_motor()}) {
    const double command = physics::hover_command(motor, prop, thrust, quad::kBusVoltage);
    const physics::MotorOutput output = settle(motor, prop, quad::still_air(command));
    EXPECT_NEAR(output.thrust_n, thrust, 2e-3 * thrust);  // within one DShot step
  }
}

TEST(MotorTest, ReactionTorqueIncludesRotorAcceleration) {
  const physics::MotorParams motor = quad::quad_motor();
  const physics::PropParams prop = quad::quad_prop();
  const physics::MotorOutput output =
      physics::step_motor(motor, prop, 1000.0, quad::still_air(1.0), 0.001);
  const double drag_only = prop.torque_coefficient * output.speed_radps * output.speed_radps;
  const double acceleration = (output.speed_radps - 1000.0) / 0.001;
  EXPECT_GT(acceleration, 0.0);
  EXPECT_NEAR(output.reaction_torque_nm, drag_only + motor.rotor_inertia_kg_m2 * acceleration,
              1e-12);
}

TEST(MotorTest, CommandIsClampedAndSpeedNeverNegative) {
  const physics::MotorParams motor = quad::quad_motor();
  const physics::PropParams prop = quad::quad_prop();
  EXPECT_LE(physics::step_motor(motor, prop, 2500.0, quad::still_air(2.0), 0.001).speed_radps,
            motor.max_speed_radps);
  EXPECT_GE(physics::step_motor(motor, prop, 1.0, quad::still_air(-1.0), 10.0).speed_radps, 0.0);
  EXPECT_GE(
      physics::step_motor(quad::dc_motor(), prop, 1.0, quad::still_air(0.0), 0.01).speed_radps,
      0.0);
}
