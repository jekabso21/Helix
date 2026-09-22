#include <cstdlib>
#include <exception>
#include <string>

#include <spdlog/spdlog.h>
#include <CLI/CLI.hpp>
#include <nlohmann/json.hpp>

#include <fpvsim/config/session.hpp>
#include <fpvsim/sim/loop.hpp>
#include <fpvsim/version.hpp>

namespace {

int run(int argc, char** argv) {
  CLI::App cli{"fpvsim simcore: physics, sensors and the Betaflight bridge"};
  cli.set_version_flag("--version", fpvsim::kVersion);

  std::string session_path;
  cli.add_option("--session", session_path, "Path to resolved session.json in the run directory")
      ->required();

  CLI11_PARSE(cli, argc, argv);

  const fpvsim::config::SessionConfig session = fpvsim::config::load_session(session_path);
  const fpvsim::sim::VehicleParams drone = fpvsim::config::load_drone(session.drone_json);
  spdlog::info("simcore {} starting: {} Hz physics, {} s, Betaflight at {}", fpvsim::kVersion,
               session.physics_rate_hz, session.duration_s, session.betaflight.host);

  const nlohmann::json info = {
      {"version", fpvsim::kVersion}, {"api_version", 1},
      {"mode", "realtime"},          {"physics_rate_hz", session.physics_rate_hz},
      {"seed", session.seed},        {"betaflight_protocol", "2026.6.2"},
      {"session", session_path}};
  const fpvsim::sim::RunSummary summary = fpvsim::sim::run_realtime(session, drone, info);
  const nlohmann::json report = {{"steps", summary.steps},
                                 {"overruns", summary.overruns},
                                 {"motor_packets", summary.motor_packets},
                                 {"malformed_packets", summary.malformed_packets},
                                 {"dropped_log_rows", summary.dropped_log_rows},
                                 {"render_states_sent", summary.render_states_sent},
                                 {"api_requests", summary.api_requests},
                                 {"final_height_m", summary.final_height_m},
                                 {"crashed", summary.crashed}};
  spdlog::info("run finished: {}", report.dump());
  return EXIT_SUCCESS;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    return run(argc, argv);
  } catch (const std::exception& error) {
    spdlog::critical("startup failed: {}", error.what());
    return EXIT_FAILURE;
  }
}
