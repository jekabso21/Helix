#include <gtest/gtest.h>

#include <stdexcept>
#include <string>

#include <fpvsim/config/session.hpp>

namespace config = fpvsim::config;

namespace {

const char* kSession = R"({
  "schema_version": 1, "seed": 42, "physics_rate_hz": 2000, "duration_s": 30.0,
  "betaflight": {"host": "127.0.0.1", "ports": {"pwm": 9002, "fdm": 9003, "rc": 9004}},
  "origin": {"lat_rad": 0.977, "lon_rad": 0.419, "altitude_m": 10.0},
  "atmosphere": {"ground_temperature_k": 288.15, "ground_pressure_pa": 101325.0},
  "spawn": {"north_m": 0.0, "east_m": 0.0, "height_agl_m": 0.0, "heading_rad": 0.0},
  "input": {"source": "altitude_hold", "rc_rate_hz": 250,
            "altitude_hold": {"target_height_m": 5.0, "climb_rate_mps": 1.0, "kp_us_per_m": 100.0,
                              "ki_us_per_m_s": 20.0, "kd_us_per_mps": 80.0,
                              "hover_throttle_us": 1350.0, "integral_limit_us": 300.0,
                              "arm_delay_s": 6.0}},
  "control_api": {"host": "127.0.0.1", "port": 7700},
  "app": {"host": "127.0.0.1", "port": 7710, "state_rate_hz": 100},
  "logging": {"rate_hz": 200, "truth_csv": "truth.csv"},
  "drone": "drone.json"
})";

const char* kDrone = R"({
  "schema_version": 1, "mass_kg": 0.5,
  "inertia_frd_kg_m2": [[0.003, 0, 0], [0, 0.003, 0], [0, 0, 0.005]],
  "motors": [
    {"bf_index": 1, "position_frd_m": [-0.08, 0.08, 0], "axis_frd": [0, 0, -1], "spin": 1,
     "rotor_inertia_kg_m2": 6e-6,
     "first_order": {"max_speed_radps": 2500, "time_constant_s": 0.03, "k_t": 1.5e-6, "k_q": 2e-8}},
    {"bf_index": 2, "position_frd_m": [0.08, 0.08, 0], "axis_frd": [0, 0, -1], "spin": -1,
     "rotor_inertia_kg_m2": 6e-6,
     "first_order": {"max_speed_radps": 2500, "time_constant_s": 0.03, "k_t": 1.5e-6, "k_q": 2e-8}}
  ],
  "imu": {"position_frd_m": [0, 0, 0]},
  "aero": {"cda_frd_m2": [0.01, 0.01, 0.02], "cop_frd_m": [0, 0, 0], "k_omega": [5e-4, 5e-4, 5e-4]},
  "contact": {"points_frd_m": [[0.08, 0.08, 0.02], [-0.08, -0.08, 0.02]],
              "stiffness_n_per_m": 3000, "damping_n_s_per_m": 30, "friction": 0.6,
              "friction_regularization_mps": 0.01, "crash_speed_mps": 6.0}
})";

std::string without(const std::string& text, const std::string& key) {
  const std::string quoted = R"(")" + key + R"(")";
  const std::size_t start = text.find(quoted);
  const std::size_t end = text.find(',', start);
  return text.substr(0, start) + text.substr(end + 1);
}

}  // namespace

TEST(ConfigTest, ParsesACompleteSession) {
  const config::SessionConfig cfg = config::parse_session(kSession, "/run/x/session.json");
  EXPECT_EQ(cfg.physics_rate_hz, 2000);
  EXPECT_EQ(cfg.betaflight.fdm_port, 9003);
  EXPECT_DOUBLE_EQ(cfg.input.altitude_hold.target_height_m, 5.0);
  EXPECT_EQ(cfg.input.rc_rate_hz, 250);
  EXPECT_EQ(cfg.drone_json, "/run/x/drone.json");
  EXPECT_EQ(cfg.logging.truth_csv, "/run/x/truth.csv");
}

TEST(ConfigTest, MissingFieldNamesItsPath) {
  try {
    config::parse_session(without(kSession, "fdm"), "session.json");
    FAIL() << "expected an error";
  } catch (const std::runtime_error& error) {
    EXPECT_NE(std::string(error.what()).find("session.json.betaflight.ports: missing field 'fdm'"),
              std::string::npos)
        << error.what();
  }
}

TEST(ConfigTest, WrongTypeNamesItsPath) {
  std::string text = kSession;
  text.replace(text.find("\"seed\": 42"), 10, R"("seed": "x")");
  EXPECT_THROW(config::parse_session(text, "session.json"), std::runtime_error);
}

TEST(ConfigTest, RejectsPhysicsRateThatIsNotAMultipleOf1000) {
  std::string text = kSession;
  text.replace(text.find("2000"), 4, "2500");
  EXPECT_THROW(config::parse_session(text, "session.json"), std::runtime_error);
}

TEST(ConfigTest, ParsesADroneInBetaflightMotorOrder) {
  const fpvsim::sim::VehicleParams drone = config::parse_drone(kDrone, "drone.json");
  EXPECT_EQ(drone.motor_count, 2U);
  EXPECT_DOUBLE_EQ(drone.mounts[1].spin, -1.0);
  EXPECT_DOUBLE_EQ(drone.motors[0].thrust_coefficient, 1.5e-6);
  EXPECT_EQ(drone.contact.point_count, 2U);
  EXPECT_DOUBLE_EQ(drone.crash_speed_mps, 6.0);
  EXPECT_NEAR(drone.mass.inertia_frd_inverse(2, 2), 200.0, 1e-9);
}

TEST(ConfigTest, RejectsMotorsOutOfOrder) {
  std::string text = kDrone;
  text.replace(text.find("\"bf_index\": 2"), 13, "\"bf_index\": 3");
  EXPECT_THROW(config::parse_drone(text, "drone.json"), std::runtime_error);
}
