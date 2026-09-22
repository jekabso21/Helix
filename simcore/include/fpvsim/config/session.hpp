#pragma once

#include <cstdint>
#include <filesystem>
#include <string>

#include <fpvsim/bridge/betaflight.hpp>
#include <fpvsim/env/atmosphere.hpp>
#include <fpvsim/input/mapping.hpp>
#include <fpvsim/pilot/altitude_hold.hpp>
#include <fpvsim/sim/vehicle.hpp>

namespace fpvsim::config {

struct Origin {
  double latitude_rad;
  double longitude_rad;
  double altitude_m;
};

struct Spawn {
  double north_m;
  double east_m;
  double height_agl_m;
  double heading_rad;
};

struct InputConfig {
  std::string source;  // "altitude_hold" or "gamepad"
  std::int64_t rc_rate_hz;
  pilot::AltitudeHoldParams altitude_hold;  // only with altitude_hold
  input::InputMapping mapping;              // only with gamepad
};

struct ControlApiConfig {
  std::string host;
  std::uint16_t port;
};

struct AppConfig {
  std::string host;
  std::uint16_t port;
  std::int64_t state_rate_hz;
};

struct LoggingConfig {
  std::int64_t rate_hz;
  std::filesystem::path truth_csv;
};

struct SessionConfig {
  std::uint64_t seed;
  std::int64_t physics_rate_hz;
  double duration_s;  // 0 = run until stopped
  bridge::betaflight::Endpoints betaflight;
  Origin origin;
  env::AtmosphereParams atmosphere;
  Spawn spawn;
  InputConfig input;
  ControlApiConfig control_api;
  AppConfig app;
  LoggingConfig logging;
  std::filesystem::path drone_json;
};

// Both throw std::runtime_error naming the file and field path on any missing or invalid field
SessionConfig load_session(const std::filesystem::path& path);
sim::VehicleParams load_drone(const std::filesystem::path& path);

SessionConfig parse_session(const std::string& json_text, const std::filesystem::path& path);
sim::VehicleParams parse_drone(const std::string& json_text, const std::filesystem::path& path);

}  // namespace fpvsim::config
