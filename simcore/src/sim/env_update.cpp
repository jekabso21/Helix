#include <fpvsim/sim/env_update.hpp>

#include <stdexcept>
#include <string>

namespace fpvsim::sim {

namespace {

double number(const nlohmann::json& node, const char* key, const std::string& where) {
  if (!node.contains(key) || !node[key].is_number()) {
    throw std::invalid_argument(where + "." + key + " must be a number");
  }
  return node[key].get<double>();
}

double at_least(double value, double minimum, const std::string& name) {
  if (value < minimum) {
    throw std::invalid_argument(name + " must not be below " + std::to_string(minimum));
  }
  return value;
}

}  // namespace

EnvUpdate env_update_from_json(const nlohmann::json& params) {
  if (!params.is_object()) {
    throw std::invalid_argument("params must be an object");
  }
  for (const auto& item : params.items()) {
    if (item.key() != "wind" && item.key() != "gust") {
      throw std::invalid_argument("unknown environment field '" + item.key() + "'");
    }
  }
  EnvUpdate update{};
  if (params.contains("wind")) {
    const nlohmann::json& wind = params["wind"];
    if (!wind.is_object()) {
      throw std::invalid_argument("wind must be an object");
    }
    if (wind.contains("mean_speed_mps") || wind.contains("mean_from_rad")) {
      update.set_mean = true;
      update.mean_speed_mps =
          at_least(number(wind, "mean_speed_mps", "wind"), 0.0, "wind.mean_speed_mps");
      update.mean_from_rad = number(wind, "mean_from_rad", "wind");
    }
    if (wind.contains("turbulence_w20_mps")) {
      update.set_turbulence = true;
      update.turbulence_w20_mps =
          at_least(number(wind, "turbulence_w20_mps", "wind"), 0.0, "wind.turbulence_w20_mps");
    }
  }
  if (params.contains("gust")) {
    const nlohmann::json& gust = params["gust"];
    if (!gust.is_object()) {
      throw std::invalid_argument("gust must be an object");
    }
    update.add_gust = true;
    update.gust_duration_s = number(gust, "duration_s", "gust");
    if (update.gust_duration_s <= 0.0) {
      throw std::invalid_argument("gust.duration_s must be positive");
    }
    update.gust_amplitude_mps =
        at_least(number(gust, "amplitude_mps", "gust"), 0.0, "gust.amplitude_mps");
    update.gust_from_rad = number(gust, "from_rad", "gust");
  }
  if (!update.set_mean && !update.set_turbulence && !update.add_gust) {
    throw std::invalid_argument("nothing to set");
  }
  return update;
}

bool apply_env_update(env::Wind& wind, const EnvUpdate& update, double time_s) {
  if (update.set_mean) {
    wind.set_mean(update.mean_speed_mps, update.mean_from_rad);
  }
  if (update.set_turbulence) {
    wind.set_turbulence(update.turbulence_w20_mps);
  }
  if (update.add_gust) {
    return wind.add_gust(env::Gust{.start_s = time_s,
                                   .duration_s = update.gust_duration_s,
                                   .amplitude_mps = update.gust_amplitude_mps,
                                   .from_rad = update.gust_from_rad});
  }
  return true;
}

}  // namespace fpvsim::sim
