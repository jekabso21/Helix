#include <gtest/gtest.h>

#include <stdexcept>

#include <nlohmann/json.hpp>

#include <fpvsim/sim/failure_request.hpp>

namespace sim = fpvsim::sim;
using nlohmann::json;

TEST(FailureRequestTest, MotorFailuresTakeABetaflightMotorNumber) {
  const sim::FailureCommand out =
      sim::failure_from_json(json::parse(R"({"type": "motor_out", "target": {"motor": 2}})"), 4);
  EXPECT_EQ(out.spec.type, sim::FailureType::kMotorOut);
  EXPECT_EQ(out.spec.motor, 1);  // 0-based inside
  EXPECT_LT(out.start_s, 0.0);
  EXPECT_LT(out.duration_s, 0.0);
  const sim::FailureCommand prop = sim::failure_from_json(json::parse(R"({
      "type": "prop_damage", "target": {"motor": 4},
      "params": {"thrust_loss": 0.3, "vibration_scale": 12},
      "start_s": 20, "duration_s": 5})"), 4);
  EXPECT_EQ(prop.spec.type, sim::FailureType::kPropDamage);
  EXPECT_EQ(prop.spec.motor, 3);
  EXPECT_DOUBLE_EQ(prop.spec.value, 0.3);
  EXPECT_DOUBLE_EQ(prop.spec.value2, 12.0);
  EXPECT_DOUBLE_EQ(prop.start_s, 20.0);
  EXPECT_DOUBLE_EQ(prop.duration_s, 5.0);
}

TEST(FailureRequestTest, SensorFailuresTakeASensorAndAxis) {
  const sim::FailureCommand bias = sim::failure_from_json(json::parse(R"({
      "type": "imu_bias", "target": {"sensor": "accel", "axis": "y"}, "params": {"step": 1.5}})"), 4);
  EXPECT_EQ(bias.spec.sensor, sim::Sensor::kAccel);
  EXPECT_EQ(bias.spec.axis, 1);
  EXPECT_DOUBLE_EQ(bias.spec.value, 1.5);
  const sim::FailureCommand stuck = sim::failure_from_json(
      json::parse(R"({"type": "imu_stuck", "target": {"sensor": "gyro", "axis": "all"}})"), 4);
  EXPECT_EQ(stuck.spec.axis, sim::kAllAxes);
  const sim::FailureCommand baro =
      sim::failure_from_json(json::parse(R"({"type": "baro_offset", "params": {"offset_pa": -300}})"), 4);
  EXPECT_DOUBLE_EQ(baro.spec.value, -300.0);
}

TEST(FailureRequestTest, RejectsRequestsThatCannotBeApplied) {
  const char* const bad[] = {
      R"({})",
      R"({"type": "gremlins"})",
      R"({"type": "motor_out"})",
      R"({"type": "motor_out", "target": {"motor": 5}})",
      R"({"type": "motor_out", "target": {"motor": 0}})",
      R"({"type": "motor_degraded", "target": {"motor": 1}, "params": {"output_gain": 1.5}})",
      R"({"type": "motor_degraded", "target": {"motor": 1}})",
      R"({"type": "esc_desync", "target": {"motor": 1}, "params": {"period_s": 0.2, "dropout_s": 0.3}})",
      R"({"type": "prop_damage", "target": {"motor": 1}, "params": {"thrust_loss": 1.2, "vibration_scale": 2}})",
      R"({"type": "imu_noise", "target": {"sensor": "magnetometer"}, "params": {"noise_scale": 2}})",
      R"({"type": "imu_bias", "target": {"sensor": "gyro", "axis": "w"}, "params": {"step": 1}})",
      R"({"type": "imu_saturation", "target": {"sensor": "gyro"}, "params": {"range_scale": 0}})",
      R"({"type": "battery_weak_cell", "params": {"drop_v": -0.2}})",
      R"({"type": "baro_stuck", "duration_s": 0})",
      R"({"type": "baro_stuck", "start_s": -3})",
  };
  for (const char* text : bad) {
    EXPECT_THROW((void)sim::failure_from_json(json::parse(text), 4), std::invalid_argument) << text;
  }
}

TEST(FailureRequestTest, ClearTakesAnIdOrAll) {
  EXPECT_EQ(sim::clear_from_json(json::parse(R"({"failure_id": 7})")).id, 7U);
  EXPECT_TRUE(sim::clear_from_json(json::parse(R"({"all": true})")).all);
  EXPECT_THROW((void)sim::clear_from_json(json::parse(R"({})")), std::invalid_argument);
  EXPECT_THROW((void)sim::clear_from_json(json::parse(R"({"failure_id": -1})")), std::invalid_argument);
}

TEST(FailureRequestTest, AListedFailureReadsLikeTheRequest) {
  const sim::FailureCommand c = sim::failure_from_json(json::parse(R"({
      "type": "prop_damage", "target": {"motor": 2},
      "params": {"thrust_loss": 0.3, "vibration_scale": 12}})"), 4);
  const sim::ActiveFailure f{.id = 4, .spec = c.spec, .start_s = 10.0, .end_s = 15.0};
  const json listed = sim::failure_to_json(f, 12.0);
  EXPECT_EQ(listed["failure_id"], 4);
  EXPECT_EQ(listed["type"], "prop_damage");
  EXPECT_EQ(listed["target"]["motor"], 2);
  EXPECT_DOUBLE_EQ(listed["params"]["thrust_loss"].get<double>(), 0.3);
  EXPECT_DOUBLE_EQ(listed["params"]["vibration_scale"].get<double>(), 12.0);
  EXPECT_TRUE(listed["in_effect"].get<bool>());
  EXPECT_FALSE(sim::failure_to_json(f, 9.0)["in_effect"].get<bool>());
  const sim::ActiveFailure open{.id = 5, .spec = c.spec, .start_s = 1.0, .end_s = -1.0};
  EXPECT_TRUE(sim::failure_to_json(open, 2.0)["end_s"].is_null());
}
