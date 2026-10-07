#pragma once

#include <nlohmann/json.hpp>

#include <fpvsim/env/wind.hpp>

namespace fpvsim::sim {

// A set_env request in SI, small and trivially copyable so it can ride in a Command
struct EnvUpdate {
  bool set_mean;
  double mean_speed_mps;
  double mean_from_rad;
  bool set_turbulence;
  double turbulence_w20_mps;
  bool add_gust;  // the gust starts at the step that applies the update
  double gust_duration_s;
  double gust_amplitude_mps;
  double gust_from_rad;
};

// {"wind": {"mean_speed_mps", "mean_from_rad", "turbulence_w20_mps"}, "gust": {"duration_s",
// "amplitude_mps", "from_rad"}}, every part optional; throws std::invalid_argument when a value
// is missing, not a number or out of range, or when nothing is set
EnvUpdate env_update_from_json(const nlohmann::json& params);
// false when the gust could not be queued
bool apply_env_update(env::Wind& wind, const EnvUpdate& update, double time_s);

}  // namespace fpvsim::sim
