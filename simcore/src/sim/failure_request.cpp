#include <fpvsim/sim/failure_request.hpp>

#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace fpvsim::sim {

namespace {

enum class Target : std::uint8_t { kNone, kMotor, kSensor, kSensorAxis };

struct Param {
  const char* name;  // nullptr: not used
  double min;
  double max;
  bool min_open;     // min itself is not allowed
};

struct TypeInfo {
  const char* name;
  FailureType type;
  Target target;
  Param value;
  Param value2;
};

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr Param kUnused{.name = nullptr, .min = 0.0, .max = 0.0, .min_open = false};

constexpr std::array<TypeInfo, 12> kTypes{{
    {"motor_out", FailureType::kMotorOut, Target::kMotor, kUnused, kUnused},
    {"motor_degraded", FailureType::kMotorDegraded, Target::kMotor,
     {"output_gain", 0.0, 1.0, false}, kUnused},
    {"esc_desync", FailureType::kEscDesync, Target::kMotor, {"period_s", 0.0, kInf, true},
     {"dropout_s", 0.0, kInf, true}},
    {"prop_damage", FailureType::kPropDamage, Target::kMotor, {"thrust_loss", 0.0, 0.99, false},
     {"vibration_scale", 0.0, 1000.0, false}},
    {"battery_weak_cell", FailureType::kBatteryWeakCell, Target::kNone, {"drop_v", 0.0, 4.2, false},
     kUnused},
    {"battery_high_resistance", FailureType::kBatteryHighResistance, Target::kNone,
     {"cell_resistance_scale", 0.0, 1000.0, true}, {"connector_add_ohm", 0.0, 10.0, false}},
    {"imu_noise", FailureType::kImuNoise, Target::kSensor, {"noise_scale", 0.0, 1000.0, false},
     kUnused},
    {"imu_bias", FailureType::kImuBias, Target::kSensorAxis, {"step", -kInf, kInf, false}, kUnused},
    {"imu_stuck", FailureType::kImuStuck, Target::kSensorAxis, kUnused, kUnused},
    {"imu_saturation", FailureType::kImuSaturation, Target::kSensor,
     {"range_scale", 0.0, 1.0, true}, kUnused},
    {"baro_stuck", FailureType::kBaroStuck, Target::kNone, kUnused, kUnused},
    {"baro_offset", FailureType::kBaroOffset, Target::kNone, {"offset_pa", -kInf, kInf, false},
     kUnused},
}};

const TypeInfo& info_for(FailureType type) {
  for (const TypeInfo& info : kTypes) {
    if (info.type == type) {
      return info;
    }
  }
  throw std::logic_error("failure type without a name");
}

const nlohmann::json& object_at(const nlohmann::json& params, const char* key) {
  static const nlohmann::json kEmpty = nlohmann::json::object();
  if (!params.contains(key)) {
    return kEmpty;
  }
  if (!params[key].is_object()) {
    throw std::invalid_argument(std::string(key) + " must be an object");
  }
  return params[key];
}

double read_param(const nlohmann::json& values, const Param& p) {
  if (p.name == nullptr) {
    return 0.0;
  }
  if (!values.contains(p.name) || !values[p.name].is_number()) {
    throw std::invalid_argument(std::string("params.") + p.name + " must be a number");
  }
  const double v = values[p.name].get<double>();
  if (!std::isfinite(v) || v < p.min || (p.min_open && v == p.min) || v > p.max) {
    throw std::invalid_argument(std::string("params.") + p.name + " is out of range");
  }
  return v;
}

double optional_time(const nlohmann::json& params, const char* key, bool positive) {
  if (!params.contains(key) || params[key].is_null()) {
    return -1.0;
  }
  if (!params[key].is_number()) {
    throw std::invalid_argument(std::string(key) + " must be a number");
  }
  const double v = params[key].get<double>();
  if (v < 0.0 || (positive && v == 0.0)) {
    throw std::invalid_argument(std::string(key) + (positive ? " must be positive" : " must not be negative"));
  }
  return v;
}

constexpr std::array<const char*, 3> kAxisNames{"x", "y", "z"};

}  // namespace

FailureCommand failure_from_json(const nlohmann::json& params, std::size_t motor_count) {
  if (!params.is_object() || !params.contains("type") || !params["type"].is_string()) {
    throw std::invalid_argument("type must be a failure name");
  }
  const std::string name = params["type"].get<std::string>();
  const TypeInfo* info = nullptr;
  for (const TypeInfo& candidate : kTypes) {
    if (name == candidate.name) {
      info = &candidate;
    }
  }
  if (info == nullptr) {
    throw std::invalid_argument("unknown failure type '" + name + "'");
  }
  FailureCommand command{.spec = {.type = info->type,
                                  .motor = 0,
                                  .sensor = Sensor::kGyro,
                                  .axis = kAllAxes,
                                  .value = 0.0,
                                  .value2 = 0.0},
                         .id = 0,
                         .start_s = optional_time(params, "start_s", false),
                         .duration_s = optional_time(params, "duration_s", true),
                         .all = false};
  const nlohmann::json& target = object_at(params, "target");
  if (info->target == Target::kMotor) {
    if (!target.contains("motor") || !target["motor"].is_number_integer()) {
      throw std::invalid_argument("target.motor must be a motor number");
    }
    const auto motor = target["motor"].get<std::int64_t>();
    if (motor < 1 || motor > static_cast<std::int64_t>(motor_count)) {
      throw std::invalid_argument("target.motor must be 1.." + std::to_string(motor_count));
    }
    command.spec.motor = static_cast<std::uint8_t>(motor - 1);
  }
  if (info->target == Target::kSensor || info->target == Target::kSensorAxis) {
    const std::string sensor = target.value("sensor", "");
    if (sensor != "gyro" && sensor != "accel") {
      throw std::invalid_argument("target.sensor must be gyro or accel");
    }
    command.spec.sensor = sensor == "gyro" ? Sensor::kGyro : Sensor::kAccel;
  }
  if (info->target == Target::kSensorAxis) {
    const std::string axis = target.value("axis", "all");
    if (axis != "all") {
      bool found = false;
      for (std::uint8_t k = 0; k < 3; ++k) {
        if (axis == kAxisNames[k]) {
          command.spec.axis = k;
          found = true;
        }
      }
      if (!found) {
        throw std::invalid_argument("target.axis must be x, y, z or all");
      }
    }
  }
  const nlohmann::json& values = object_at(params, "params");
  command.spec.value = read_param(values, info->value);
  command.spec.value2 = read_param(values, info->value2);
  if (info->type == FailureType::kEscDesync && command.spec.value2 >= command.spec.value) {
    throw std::invalid_argument("params.dropout_s must be shorter than params.period_s");
  }
  return command;
}

FailureCommand clear_from_json(const nlohmann::json& params) {
  FailureCommand command{.spec = {.type = FailureType::kBaroStuck,
                                  .motor = 0,
                                  .sensor = Sensor::kGyro,
                                  .axis = kAllAxes,
                                  .value = 0.0,
                                  .value2 = 0.0},
                         .id = 0,
                         .start_s = -1.0,
                         .duration_s = -1.0,
                         .all = false};
  if (params.is_object() && params.contains("all") && params["all"] == true) {
    command.all = true;
    return command;
  }
  if (!params.is_object() || !params.contains("failure_id") ||
      !params["failure_id"].is_number_unsigned()) {
    throw std::invalid_argument("give failure_id or all: true");
  }
  command.id = params["failure_id"].get<std::uint32_t>();
  return command;
}

nlohmann::json failure_to_json(const ActiveFailure& failure, double time_s) {
  const TypeInfo& info = info_for(failure.spec.type);
  nlohmann::json target = nlohmann::json::object();
  if (info.target == Target::kMotor) {
    target["motor"] = failure.spec.motor + 1;
  }
  if (info.target == Target::kSensor || info.target == Target::kSensorAxis) {
    target["sensor"] = failure.spec.sensor == Sensor::kGyro ? "gyro" : "accel";
  }
  if (info.target == Target::kSensorAxis) {
    target["axis"] = failure.spec.axis == kAllAxes ? "all" : kAxisNames[failure.spec.axis];
  }
  nlohmann::json values = nlohmann::json::object();
  if (info.value.name != nullptr) {
    values[info.value.name] = failure.spec.value;
  }
  if (info.value2.name != nullptr) {
    values[info.value2.name] = failure.spec.value2;
  }
  const bool in_effect =
      time_s >= failure.start_s && (failure.end_s < 0.0 || time_s < failure.end_s);
  return {{"failure_id", failure.id},
          {"type", info.name},
          {"target", target},
          {"params", values},
          {"start_s", failure.start_s},
          {"end_s", failure.end_s < 0.0 ? nlohmann::json(nullptr) : nlohmann::json(failure.end_s)},
          {"in_effect", in_effect}};
}

}  // namespace fpvsim::sim
