#include <gtest/gtest.h>

#include <fpvsim/physics/battery.hpp>

#include "test_quad.hpp"

namespace physics = fpvsim::physics;
namespace quad = fpvsim::testing;

TEST(BatteryTest, OpenCircuitVoltageInterpolatesTheCurveAndClampsSoc) {
  const physics::BatteryParams params = quad::quad_battery();
  EXPECT_DOUBLE_EQ(physics::open_circuit_voltage(params, 1.0), 4.20);
  EXPECT_DOUBLE_EQ(physics::open_circuit_voltage(params, 0.0), 3.30);
  EXPECT_NEAR(physics::open_circuit_voltage(params, 0.95), 4.15, 1e-12);
  EXPECT_DOUBLE_EQ(physics::open_circuit_voltage(params, 1.7), 4.20);
  EXPECT_DOUBLE_EQ(physics::open_circuit_voltage(params, -1.0), 3.30);
}

TEST(BatteryTest, NoLoadGivesTheOpenCircuitPackVoltage) {
  physics::BatteryParams params = quad::quad_battery();
  params.avionics_current_a = 0.0;
  const physics::BatteryState state = physics::initial_battery(params);
  EXPECT_DOUBLE_EQ(state.bus_voltage_v, 6.0 * 4.20);
  const physics::BatteryState next = physics::step_battery(params, state, 0.0, 0.001);
  EXPECT_DOUBLE_EQ(next.bus_voltage_v, 6.0 * 4.20);
  EXPECT_DOUBLE_EQ(next.current_a, 0.0);
  EXPECT_FALSE(next.cutoff);
}

TEST(BatteryTest, SagEqualsCurrentTimesResistance) {
  physics::BatteryParams params = quad::quad_battery();
  params.avionics_current_a = 0.0;
  const physics::BatteryState state = physics::initial_battery(params);
  const physics::BatteryState loaded = physics::step_battery(params, state, 50.0, 1e-6);
  const double expected_sag =
      50.0 * (6.0 * params.cell_resistance_ohm + params.connector_resistance_ohm);
  EXPECT_NEAR(loaded.bus_voltage_v, 6.0 * 4.20 - expected_sag, 1e-6);
  EXPECT_DOUBLE_EQ(loaded.current_a, 50.0);
}

TEST(BatteryTest, SocDropsByChargeOverCapacity) {
  physics::BatteryParams params = quad::quad_battery();
  params.avionics_current_a = 0.0;
  physics::BatteryState state = physics::initial_battery(params);
  for (int i = 0; i < 60000; ++i) {  // 60 s at 33 A = 0.55 Ah, half the pack
    state = physics::step_battery(params, state, 33.0, 0.001);
  }
  EXPECT_NEAR(state.soc, 0.5, 1e-9);
  EXPECT_NEAR(state.consumed_ah, 0.55, 1e-9);
}

TEST(BatteryTest, AvionicsCurrentAlwaysFlowsAndCutoffHasHysteresis) {
  physics::BatteryParams params = quad::quad_battery();
  params.esc_cutoff_v = 24.0;
  physics::BatteryState state = physics::initial_battery(params);
  state = physics::step_battery(params, state, 0.0, 0.001);
  EXPECT_DOUBLE_EQ(state.current_a, params.avionics_current_a);
  EXPECT_FALSE(state.cutoff);
  state = physics::step_battery(params, state, 80.0, 0.001);
  EXPECT_TRUE(state.cutoff);
  state = physics::step_battery(params, state, 30.0, 0.001);  // 25.2 - 30 * 0.05 = 23.7 V
  EXPECT_TRUE(state.cutoff);                                  // still below cutoff + 1 V
  state = physics::step_battery(params, state, 0.0, 0.001);
  EXPECT_FALSE(state.cutoff);  // recovered above 25 V
}

TEST(BatteryTest, SolvedBusVoltageIsConsistentWithTheCurrentItCauses) {
  physics::BatteryParams params = quad::quad_battery();
  params.avionics_current_a = 0.0;
  const physics::BatteryState state = physics::initial_battery(params);
  const double conductance = 22.2;  // four DC motors at full throttle, 0.18 ohm each
  const double fixed = -300.0;      // back-EMF term
  const double v = physics::solve_bus_voltage(params, state, fixed, conductance);
  const double current = fixed + conductance * v;
  const double r = 6.0 * params.cell_resistance_ohm + params.connector_resistance_ohm;
  EXPECT_NEAR(v, 6.0 * 4.20 - r * current, 1e-9);
  EXPECT_LT(v, 6.0 * 4.20);
  EXPECT_GT(v, 15.0);
}

TEST(BatteryTest, TransientBranchRelaxesWithItsTimeConstant) {
  physics::BatteryParams params = quad::quad_battery();
  params.avionics_current_a = 0.0;
  params.rc_resistance_ohm = 0.004;
  params.rc_capacitance_f = 50.0;  // tau 0.2 s
  physics::BatteryState state = physics::initial_battery(params);
  for (int i = 0; i < 200; ++i) {
    state = physics::step_battery(params, state, 20.0, 0.001);
  }
  const double target = 20.0 * params.rc_resistance_ohm;
  EXPECT_NEAR(state.v_rc / target, 1.0 - std::exp(-1.0), 1e-9);
  const double steady = 6.0 * (physics::open_circuit_voltage(params, state.soc) -
                               20.0 * params.cell_resistance_ohm - state.v_rc) -
                        20.0 * params.connector_resistance_ohm;
  EXPECT_NEAR(state.bus_voltage_v, steady, 1e-9);
}
