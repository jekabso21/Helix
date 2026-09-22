#include <fpvsim/config/session.hpp>

#include <fstream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <vector>

#include <nlohmann/json.hpp>

#include <fpvsim/input/mapping_json.hpp>

namespace fpvsim::config {

namespace {

using nlohmann::json;

inline constexpr int kSchemaVersion = 1;
constexpr int kDroneSchemaVersion = 2;

class Reader {
 public:
  Reader(const json& node, std::string path) : node_(node), path_(std::move(path)) {}

  [[nodiscard]] Reader at(const std::string& key) const {
    if (!node_.is_object() || !node_.contains(key)) {
      throw std::runtime_error(path_ + ": missing field '" + key + "'");
    }
    return {node_[key], path_ + "." + key};
  }

  [[nodiscard]] Reader index(std::size_t i) const {
    if (!node_.is_array() || i >= node_.size()) {
      throw std::runtime_error(path_ + ": missing element " + std::to_string(i));
    }
    return {node_[i], path_ + "[" + std::to_string(i) + "]"};
  }

  [[nodiscard]] const json& raw() const { return node_; }

  [[nodiscard]] bool has(const std::string& key) const {
    return node_.is_object() && node_.contains(key);
  }

  [[nodiscard]] std::vector<std::string> keys() const {
    if (!node_.is_object()) {
      throw std::runtime_error(path_ + ": expected an object");
    }
    std::vector<std::string> keys;
    for (const auto& item : node_.items()) {
      keys.push_back(item.key());
    }
    return keys;
  }

  [[nodiscard]] std::size_t size() const {
    if (!node_.is_array()) {
      throw std::runtime_error(path_ + ": expected an array");
    }
    return node_.size();
  }

  template <typename T>
  [[nodiscard]] T get() const {
    try {
      return node_.get<T>();
    } catch (const json::exception& error) {
      throw std::runtime_error(path_ + ": " + error.what());
    }
  }

  [[nodiscard]] double number() const {
    if (!node_.is_number()) {
      throw std::runtime_error(path_ + ": expected a number");
    }
    return node_.get<double>();
  }

  [[nodiscard]] Eigen::Vector3d vector3() const {
    if (size() != 3) {
      throw std::runtime_error(path_ + ": expected 3 numbers");
    }
    return {index(0).number(), index(1).number(), index(2).number()};
  }

  [[nodiscard]] Eigen::Matrix3d matrix3() const {
    Eigen::Matrix3d m;
    for (std::size_t r = 0; r < 3; ++r) {
      m.row(static_cast<Eigen::Index>(r)) = index(r).vector3().transpose();
    }
    return m;
  }

 private:
  const json& node_;
  std::string path_;
};

json parse_text(const std::string& text, const std::filesystem::path& path) {
  try {
    return json::parse(text);
  } catch (const json::exception& error) {
    throw std::runtime_error(path.string() + ": " + error.what());
  }
}

std::string read_file(const std::filesystem::path& path) {
  std::ifstream file(path);
  if (!file) {
    throw std::runtime_error("cannot read " + path.string());
  }
  std::stringstream buffer;
  buffer << file.rdbuf();
  return buffer.str();
}

void check_schema(const Reader& root, int expected = kSchemaVersion) {
  const int version = root.at("schema_version").get<int>();
  if (version != expected) {
    throw std::runtime_error("unsupported schema_version " + std::to_string(version));
  }
}

}  // namespace

SessionConfig parse_session(const std::string& json_text, const std::filesystem::path& path) {
  const json document = parse_text(json_text, path);
  const Reader root(document, path.filename().string());
  check_schema(root);
  SessionConfig cfg{};
  cfg.seed = root.at("seed").get<std::uint64_t>();
  cfg.physics_rate_hz = root.at("physics_rate_hz").get<std::int64_t>();
  if (cfg.physics_rate_hz <= 0 || cfg.physics_rate_hz % 1000 != 0 ||
      1'000'000'000 % cfg.physics_rate_hz != 0) {
    throw std::runtime_error("physics_rate_hz must be a multiple of 1000 that divides 1e9");
  }
  cfg.duration_s = root.at("duration_s").number();

  const Reader bf = root.at("betaflight");
  cfg.betaflight.host = bf.at("host").get<std::string>();
  cfg.betaflight.pwm_port = bf.at("ports").at("pwm").get<std::uint16_t>();
  cfg.betaflight.fdm_port = bf.at("ports").at("fdm").get<std::uint16_t>();
  cfg.betaflight.rc_port = bf.at("ports").at("rc").get<std::uint16_t>();
  const Reader esc = bf.at("esc");
  cfg.esc = {.enabled = esc.at("enabled").get<bool>(),
             .host = cfg.betaflight.host,
             .request_port = esc.at("request_port").get<std::uint16_t>(),
             .uart_port = esc.at("uart_port").get<std::uint16_t>()};

  const Reader origin = root.at("origin");
  cfg.origin = {.latitude_rad = origin.at("lat_rad").number(),
                .longitude_rad = origin.at("lon_rad").number(),
                .altitude_m = origin.at("altitude_m").number()};

  const Reader atmosphere = root.at("atmosphere");
  cfg.atmosphere = {.ground_temperature_k = atmosphere.at("ground_temperature_k").number(),
                    .ground_pressure_pa = atmosphere.at("ground_pressure_pa").number()};

  const Reader spawn = root.at("spawn");
  cfg.spawn = {.north_m = spawn.at("north_m").number(),
               .east_m = spawn.at("east_m").number(),
               .height_agl_m = spawn.at("height_agl_m").number(),
               .heading_rad = spawn.at("heading_rad").number()};

  const Reader input = root.at("input");
  cfg.input.source = input.at("source").get<std::string>();
  cfg.input.rc_rate_hz = input.at("rc_rate_hz").get<std::int64_t>();
  if (cfg.input.rc_rate_hz <= 0 || cfg.physics_rate_hz % cfg.input.rc_rate_hz != 0) {
    throw std::runtime_error("input.rc_rate_hz must divide physics_rate_hz");
  }
  if (cfg.input.source == "altitude_hold") {
    const Reader hold = input.at("altitude_hold");
    cfg.input.altitude_hold = {.target_height_m = hold.at("target_height_m").number(),
                               .climb_rate_mps = hold.at("climb_rate_mps").number(),
                               .kp_us_per_m = hold.at("kp_us_per_m").number(),
                               .ki_us_per_m_s = hold.at("ki_us_per_m_s").number(),
                               .kd_us_per_mps = hold.at("kd_us_per_mps").number(),
                               .hover_throttle_us = hold.at("hover_throttle_us").number(),
                               .integral_limit_us = hold.at("integral_limit_us").number(),
                               .arm_delay_s = hold.at("arm_delay_s").number()};
  } else if (cfg.input.source == "gamepad") {
    cfg.input.mapping = input::mapping_from_json(input.at("mapping").raw());
  } else {
    throw std::runtime_error("input.source must be 'altitude_hold' or 'gamepad'");
  }

  const Reader control_api = root.at("control_api");
  cfg.control_api = {.host = control_api.at("host").get<std::string>(),
                     .port = control_api.at("port").get<std::uint16_t>()};
  const Reader app = root.at("app");
  cfg.app = {.host = app.at("host").get<std::string>(),
             .port = app.at("port").get<std::uint16_t>(),
             .state_rate_hz = app.at("state_rate_hz").get<std::int64_t>()};
  if (cfg.app.state_rate_hz <= 0 || cfg.physics_rate_hz % cfg.app.state_rate_hz != 0) {
    throw std::runtime_error("app.state_rate_hz must divide physics_rate_hz");
  }

  const Reader logging = root.at("logging");
  cfg.logging.rate_hz = logging.at("rate_hz").get<std::int64_t>();
  if (cfg.logging.rate_hz <= 0 || cfg.physics_rate_hz % cfg.logging.rate_hz != 0) {
    throw std::runtime_error("logging.rate_hz must divide physics_rate_hz");
  }
  cfg.logging.truth_csv = path.parent_path() / logging.at("truth_csv").get<std::string>();
  cfg.drone_json = path.parent_path() / root.at("drone").get<std::string>();
  return cfg;
}

sim::VehicleParams parse_drone(const std::string& json_text, const std::filesystem::path& path) {
  const json document = parse_text(json_text, path);
  const Reader root(document, path.filename().string());
  check_schema(root, kDroneSchemaVersion);
  sim::VehicleParams params{};
  params.mass = physics::make_mass_properties(root.at("mass_kg").number(),
                                              root.at("inertia_frd_kg_m2").matrix3());

  const Reader motors = root.at("motors");
  params.motor_count = motors.size();
  if (params.motor_count == 0 || params.motor_count > physics::kMaxMotors) {
    throw std::runtime_error("motors: expected 1 to " + std::to_string(physics::kMaxMotors));
  }
  for (std::size_t i = 0; i < params.motor_count; ++i) {
    const Reader motor = motors.index(i);
    const auto bf_index = motor.at("bf_index").get<std::size_t>();
    if (bf_index != i + 1) {
      throw std::runtime_error("motors must be listed in Betaflight order starting at 1");
    }
    params.mounts[i] = {.position_frd = motor.at("position_frd_m").vector3(),
                        .axis_frd = motor.at("axis_frd").vector3().normalized(),
                        .spin = motor.at("spin").number()};
    const Reader m = motor.at("motor");
    const auto model_name = m.at("model").get<std::string>();
    if (model_name != "first_order" && model_name != "dc") {
      throw std::runtime_error("motor.model must be first_order or dc");
    }
    params.motors[i] = {
        .model = model_name == "dc" ? physics::MotorModel::kDc : physics::MotorModel::kFirstOrder,
        .max_speed_radps = m.at("max_speed_radps").number(),
        .time_constant_s = m.at("time_constant_s").number(),
        .reference_voltage_v = m.at("reference_voltage_v").number(),
        .kv_radps_per_v = m.at("kv_radps_per_v").number(),
        .resistance_ohm = m.at("resistance_ohm").number(),
        .no_load_current_a = m.at("no_load_current_a").number(),
        .brake_current_a = m.at("brake_current_a").number(),
        .rotor_inertia_kg_m2 = m.at("rotor_inertia_kg_m2").number(),
        .pole_pairs = m.at("pole_pairs").get<int>()};
    const Reader p = motor.at("prop");
    params.props[i] = {.thrust_coefficient = p.at("k_t").number(),
                       .torque_coefficient = p.at("k_q").number(),
                       .reference_density_kg_m3 = p.at("rho_ref_kg_m3").number(),
                       .radius_m = p.at("radius_m").number(),
                       .pitch_m = p.at("pitch_m").number(),
                       .inflow_coefficient = p.at("inflow_coefficient").number(),
                       .rotor_drag_coefficient = p.at("rotor_drag_coefficient").number(),
                       .blades = p.at("blades").get<int>()};
  }

  const Reader battery = root.at("battery");
  params.battery = {.cells = battery.at("cells").get<int>(),
                    .capacity_ah = battery.at("capacity_ah").number(),
                    .cell_resistance_ohm = battery.at("cell_resistance_ohm").number(),
                    .connector_resistance_ohm = battery.at("connector_resistance_ohm").number(),
                    .avionics_current_a = battery.at("avionics_current_a").number(),
                    .esc_cutoff_v = battery.at("esc_cutoff_v").number(),
                    .initial_soc = battery.at("initial_soc").number(),
                    .rc_resistance_ohm = battery.at("rc_resistance_ohm").number(),
                    .rc_capacitance_f = battery.at("rc_capacitance_f").number(),
                    .ocv_v = {}};
  const Reader ocv = battery.at("ocv_v");
  if (ocv.size() != physics::kOcvPoints) {
    throw std::runtime_error("battery.ocv_v: expected " + std::to_string(physics::kOcvPoints) +
                             " values");
  }
  for (std::size_t i = 0; i < physics::kOcvPoints; ++i) {
    params.battery.ocv_v[i] = ocv.index(i).number();
  }

  const Reader imu_noise = root.at("sensors").at("imu");
  const Reader baro = root.at("sensors").at("baro");
  // sample rate and seed come from the session; the loop fills them before building the vehicle
  params.imu_noise = {
      .sample_rate_hz = 0.0,
      .gyro_noise_density = imu_noise.at("gyro_noise_density_radps_rthz").number(),
      .gyro_bias_walk = imu_noise.at("gyro_bias_walk_radps2_rthz").number(),
      .gyro_range_radps = imu_noise.at("gyro_range_radps").number(),
      .accel_noise_density = imu_noise.at("accel_noise_density_mps2_rthz").number(),
      .accel_bias_walk = imu_noise.at("accel_bias_walk_mps3_rthz").number(),
      .accel_range_mps2 = imu_noise.at("accel_range_mps2").number(),
      .vibration_imbalance = imu_noise.at("vibration_imbalance_mps2_per_radps2").number(),
      .vibration_harmonic2 = imu_noise.at("vibration_harmonic2").number(),
      .vibration_blade_pass = imu_noise.at("vibration_blade_pass").number(),
      .vibration_gyro_gain = imu_noise.at("vibration_gyro_gain_radps_per_mps2").number(),
      .baro_noise_pa = baro.at("noise_pa").number(),
      .baro_bias_pa = baro.at("bias_pa").number(),
      .seed = 0};

  const Reader aero = root.at("aero");
  params.aero = {.drag_area_frd_m2 = aero.at("cda_frd_m2").vector3(),
                 .center_of_pressure_frd = aero.at("cop_frd_m").vector3(),
                 .angular_damping = aero.at("k_omega").vector3()};

  const Reader contact = root.at("contact");
  const Reader points = contact.at("points_frd_m");
  params.contact.point_count = points.size();
  if (params.contact.point_count == 0 || params.contact.point_count > physics::kMaxContactPoints) {
    throw std::runtime_error("contact.points_frd_m: expected 1 to " +
                             std::to_string(physics::kMaxContactPoints));
  }
  for (std::size_t i = 0; i < params.contact.point_count; ++i) {
    params.contact.points_frd[i] = points.index(i).vector3();
  }
  params.contact.stiffness_n_per_m = contact.at("stiffness_n_per_m").number();
  params.contact.damping_n_s_per_m = contact.at("damping_n_s_per_m").number();
  params.contact.friction = contact.at("friction").number();
  params.contact.friction_regularization_mps = contact.at("friction_regularization_mps").number();
  params.crash_speed_mps = contact.at("crash_speed_mps").number();
  params.imu_offset_frd = root.at("imu").at("position_frd_m").vector3();
  return params;
}

SessionConfig load_session(const std::filesystem::path& path) {
  return parse_session(read_file(path), path);
}

sim::VehicleParams load_drone(const std::filesystem::path& path) {
  return parse_drone(read_file(path), path);
}

}  // namespace fpvsim::config
