#include <gtest/gtest.h>

#include <fpvsim/sim/failures.hpp>

namespace sim = fpvsim::sim;

namespace {

sim::FailureSpec spec(sim::FailureType type, std::uint8_t motor = 0, double value = 0.0,
                      double value2 = 0.0) {
  return sim::FailureSpec{.type = type,
                          .motor = motor,
                          .sensor = sim::Sensor::kGyro,
                          .axis = sim::kAllAxes,
                          .value = value,
                          .value2 = value2};
}

sim::ActiveFailure now(std::uint32_t id, const sim::FailureSpec& s) {
  return sim::ActiveFailure{.id = id, .spec = s, .start_s = 0.0, .end_s = -1.0};
}

}  // namespace

TEST(FailuresTest, NothingInjectedIsHealthy) {
  const sim::FailureModifiers m = sim::modifiers_from({}, 5.0);
  for (const sim::MotorFault& motor : m.motors) {
    EXPECT_EQ(motor.output_gain, 1.0);
    EXPECT_EQ(motor.thrust_scale, 1.0);
    EXPECT_EQ(motor.desync_period_s, 0.0);
  }
  EXPECT_EQ(m.cell_resistance_scale, 1.0);
  EXPECT_EQ(m.weak_cell_drop_v, 0.0);
  EXPECT_EQ(m.imu.gyro.noise_scale, 1.0);
  EXPECT_FALSE(m.imu.baro_stuck);
}

TEST(FailuresTest, MotorFailuresHitOnlyTheirMotor) {
  const std::array failures{
      now(1, spec(sim::FailureType::kMotorOut, 1)),
      now(2, spec(sim::FailureType::kMotorDegraded, 2, 0.6)),
      now(3, spec(sim::FailureType::kPropDamage, 3, 0.25, 8.0)),
      now(4, spec(sim::FailureType::kEscDesync, 0, 0.5, 0.05)),
  };
  const sim::FailureModifiers m = sim::modifiers_from(failures, 1.0);
  EXPECT_EQ(m.motors[1].output_gain, 0.0);
  EXPECT_EQ(m.motors[2].output_gain, 0.6);
  EXPECT_EQ(m.motors[3].thrust_scale, 0.75);
  EXPECT_EQ(m.motors[3].torque_scale, 0.75);
  EXPECT_EQ(m.imu.imbalance_scale[3], 8.0);
  EXPECT_EQ(m.imu.imbalance_scale[2], 1.0);
  EXPECT_EQ(m.motors[0].desync_period_s, 0.5);
  EXPECT_EQ(m.motors[0].output_gain, 1.0);
}

TEST(FailuresTest, OverlappingFailuresCombine) {
  const std::array failures{
      now(1, spec(sim::FailureType::kMotorDegraded, 0, 0.5)),
      now(2, spec(sim::FailureType::kMotorDegraded, 0, 0.5)),
      now(3, spec(sim::FailureType::kBatteryWeakCell, 0, 0.3)),
      now(4, spec(sim::FailureType::kBatteryWeakCell, 0, 0.2)),
      now(5, spec(sim::FailureType::kBatteryHighResistance, 0, 3.0, 0.01)),
  };
  const sim::FailureModifiers m = sim::modifiers_from(failures, 1.0);
  EXPECT_DOUBLE_EQ(m.motors[0].output_gain, 0.25);
  EXPECT_DOUBLE_EQ(m.weak_cell_drop_v, 0.5);
  EXPECT_DOUBLE_EQ(m.cell_resistance_scale, 3.0);
  EXPECT_DOUBLE_EQ(m.connector_resistance_add_ohm, 0.01);
}

TEST(FailuresTest, SensorFailuresTargetASensorAndAxis) {
  sim::FailureSpec bias = spec(sim::FailureType::kImuBias, 0, 0.4);
  bias.sensor = sim::Sensor::kAccel;
  bias.axis = 1;
  sim::FailureSpec stuck = spec(sim::FailureType::kImuStuck);
  stuck.axis = sim::kAllAxes;
  sim::FailureSpec noisy = spec(sim::FailureType::kImuNoise, 0, 4.0);
  sim::FailureSpec saturated = spec(sim::FailureType::kImuSaturation, 0, 0.3);
  saturated.sensor = sim::Sensor::kAccel;
  const std::array failures{now(1, bias), now(2, stuck), now(3, noisy), now(4, saturated),
                            now(5, spec(sim::FailureType::kBaroOffset, 0, -120.0)),
                            now(6, spec(sim::FailureType::kBaroStuck))};
  const sim::FailureModifiers m = sim::modifiers_from(failures, 1.0);
  EXPECT_EQ(m.imu.accel.bias_step, Eigen::Vector3d(0.0, 0.4, 0.0));
  EXPECT_TRUE(m.imu.gyro.stuck[0] && m.imu.gyro.stuck[1] && m.imu.gyro.stuck[2]);
  EXPECT_FALSE(m.imu.accel.stuck[0]);
  EXPECT_EQ(m.imu.gyro.noise_scale, 4.0);
  EXPECT_EQ(m.imu.accel.range_scale, 0.3);
  EXPECT_EQ(m.imu.baro_offset_pa, -120.0);
  EXPECT_TRUE(m.imu.baro_stuck);
}

TEST(FailuresTest, ScheduledFailuresStartAndEndOnTime) {
  sim::FailureSet set;
  ASSERT_TRUE(set.add(7, spec(sim::FailureType::kMotorOut, 0), 10.0, 12.0));
  EXPECT_TRUE(set.update(0.0));  // first call always reports
  EXPECT_EQ(set.modifiers(5.0).motors[0].output_gain, 1.0);
  EXPECT_FALSE(set.update(9.0));
  EXPECT_TRUE(set.update(10.0));  // it starts
  EXPECT_EQ(set.modifiers(10.0).motors[0].output_gain, 0.0);
  EXPECT_FALSE(set.update(11.0));
  EXPECT_TRUE(set.update(12.5));  // it ended and is dropped
  EXPECT_TRUE(set.held().empty());
  EXPECT_EQ(set.modifiers(12.5).motors[0].output_gain, 1.0);
}

TEST(FailuresTest, FailuresCanBeClearedAndTheSetIsBounded) {
  sim::FailureSet set;
  for (std::uint32_t id = 1; id <= sim::kMaxFailures; ++id) {
    ASSERT_TRUE(set.add(id, spec(sim::FailureType::kBaroStuck), 0.0, -1.0));
  }
  EXPECT_FALSE(set.add(99, spec(sim::FailureType::kBaroStuck), 0.0, -1.0));
  EXPECT_TRUE(set.remove(3));
  EXPECT_FALSE(set.remove(3));
  EXPECT_EQ(set.held().size(), sim::kMaxFailures - 1);
  EXPECT_TRUE(set.update(0.0));
  set.clear();
  EXPECT_TRUE(set.update(0.0));
  EXPECT_TRUE(set.held().empty());
}

TEST(FailuresTest, DesyncDropsOutForPartOfEachPeriod) {
  const sim::MotorFault fault{.output_gain = 1.0,
                              .thrust_scale = 1.0,
                              .torque_scale = 1.0,
                              .desync_period_s = 0.5,
                              .desync_dropout_s = 0.1};
  EXPECT_TRUE(sim::desync_dropped_out(fault, 0.05));
  EXPECT_FALSE(sim::desync_dropped_out(fault, 0.2));
  EXPECT_TRUE(sim::desync_dropped_out(fault, 1.02));
  sim::MotorFault healthy = fault;
  healthy.desync_period_s = 0.0;
  EXPECT_FALSE(sim::desync_dropped_out(healthy, 0.05));
}
