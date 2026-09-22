#include <fpvsim/sim/loop.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <numbers>
#include <thread>

#include <fpvsim/bridge/betaflight.hpp>
#include <fpvsim/env/atmosphere.hpp>
#include <fpvsim/io/io_thread.hpp>
#include <fpvsim/pilot/altitude_hold.hpp>
#include <fpvsim/sim/snapshot.hpp>
#include <fpvsim/sim/step_stats.hpp>
#include <fpvsim/sim/vehicle.hpp>
#include <fpvsim/sim_time.hpp>

namespace fpvsim::sim {

namespace {

namespace bf = bridge::betaflight;

constexpr std::int64_t kFdmRateHz = 1000;
constexpr double kArmedCommandThreshold = 0.01;
constexpr double kRadPerSecToRpm = 60.0 / (2.0 * std::numbers::pi);

bool any_motor_running(const bf::MotorCommands& commands) {
  return std::ranges::any_of(commands.command, [](double c) { return c > kArmedCommandThreshold; });
}

struct LoopCounters {
  std::uint64_t overruns = 0;
  StepStats stats;
};

Snapshot make_snapshot(SimTime t, std::int64_t step_index, const Vehicle& vehicle,
                       const StepResult& result, const bf::MotorCommands& commands,
                       const bf::RcChannels& rc, bool paused, double air_density,
                       const LoopCounters& counters, const bf::LinkCounters& link) {
  const VehicleState& state = vehicle.state();
  const auto& b = state.body;
  Snapshot s{};
  s.sim_time_ns = t.ns;
  s.step_index = step_index;
  s.position_ned = {b.position_ned.x(), b.position_ned.y(), b.position_ned.z()};
  s.velocity_ned = {b.velocity_ned.x(), b.velocity_ned.y(), b.velocity_ned.z()};
  s.q_ned_from_frd = {b.q_ned_from_frd.w(), b.q_ned_from_frd.x(), b.q_ned_from_frd.y(),
                      b.q_ned_from_frd.z()};
  s.angular_rate_frd = {b.angular_rate_frd.x(), b.angular_rate_frd.y(), b.angular_rate_frd.z()};
  s.motor_count = static_cast<std::uint8_t>(vehicle.params().motor_count);
  for (std::size_t i = 0; i < vehicle.params().motor_count; ++i) {
    s.motor_command[i] = i < bf::kMotorCount ? commands.command[i] : 0.0;
    s.motor_rpm[i] = state.motor_speed_radps[i] * kRadPerSecToRpm;
    s.motor_thrust_n[i] = result.motors[i].thrust_n;
  }
  s.rc_channels_us = rc;
  s.armed = any_motor_running(commands);
  s.crashed = state.crashed;
  s.touching = result.touching;
  s.paused = paused;
  s.air_density_kg_m3 = air_density;
  s.overruns = counters.overruns;
  s.motor_packets = link.motor_packets;
  s.malformed_packets = link.malformed_packets;
  s.step_time_mean_us = counters.stats.mean_us();
  s.step_time_p99_us = counters.stats.p99_us();
  s.step_time_max_us = counters.stats.max_us();
  return s;
}

// Applies queued control API commands at the start of a step; returns false on shutdown
bool apply_commands(io::CommandQueue& commands_in, io::ResultQueue& results_out, SimTime t,
                    Vehicle& vehicle, const physics::RigidBodyState& spawn,
                    pilot::AltitudeHoldPilot& autopilot, bf::MotorCommands& commands,
                    MotorCommandArray& motor_commands, bool& paused) {
  bool running = true;
  Command command{};
  while (commands_in.try_pop(command)) {
    switch (command.type) {
      case CommandType::kReset:
        vehicle.reset(spawn);
        autopilot.reset(to_seconds(t));
        commands = {};
        motor_commands = {};
        break;
      case CommandType::kPause:
        paused = true;
        break;
      case CommandType::kResume:
        paused = false;
        break;
      case CommandType::kShutdown:
        running = false;
        break;
    }
    results_out.try_push(CommandResult{
        .client = command.client, .request_id = command.request_id, .applied_at_ns = t.ns});
  }
  return running;
}

}  // namespace

RunSummary run_realtime(const config::SessionConfig& session, const VehicleParams& drone,
                        const nlohmann::json& info) {
  const SimTime step = step_from_rate_hz(session.physics_rate_hz).value_or(SimTime{});
  const double dt_s = to_seconds(step);
  const std::int64_t fdm_every = session.physics_rate_hz / kFdmRateHz;
  const std::int64_t rc_every = session.physics_rate_hz / session.input.rc_rate_hz;
  const std::int64_t total_steps =
      session.duration_s > 0.0
          ? static_cast<std::int64_t>(
                std::llround(session.duration_s * static_cast<double>(session.physics_rate_hz)))
          : -1;

  const physics::RigidBodyState spawn =
      spawn_state(drone, session.spawn.north_m, session.spawn.east_m, session.spawn.height_agl_m,
                  session.spawn.heading_rad);
  Vehicle vehicle(drone, spawn);
  bf::BetaflightLink link(session.betaflight);
  pilot::AltitudeHoldPilot autopilot(session.input.altitude_hold);

  io::SnapshotQueue snapshots;
  io::CommandQueue commands_in;
  io::ResultQueue results_out;
  io::IoThread io(io::IoConfig{.api_host = session.control_api.host,
                               .api_port = session.control_api.port,
                               .app_host = session.app.host,
                               .app_port = session.app.port,
                               .physics_rate_hz = session.physics_rate_hz,
                               .state_rate_hz = session.app.state_rate_hz,
                               .log_rate_hz = session.logging.rate_hz,
                               .truth_csv = session.logging.truth_csv,
                               .info = info},
                  snapshots, commands_in, results_out);

  bf::MotorCommands commands{};
  bf::RcChannels rc{};
  rc.fill(1000);
  MotorCommandArray motor_commands{};
  bf::FdmInput last_fdm{};
  bool have_fdm = false;
  StepResult result{};
  LoopCounters counters;
  std::uint64_t dropped_snapshots = 0;
  bool paused = false;
  bool running = true;
  SimTime t{};
  std::int64_t step_index = 0;
  std::int64_t wall_tick = 0;

  const auto wall_start = std::chrono::steady_clock::now();
  while (running && (total_steps < 0 || step_index < total_steps)) {
    const auto step_begin = std::chrono::steady_clock::now();

    running = apply_commands(commands_in, results_out, t, vehicle, spawn, autopilot, commands,
                             motor_commands, paused);

    link.poll_motors(commands);
    for (std::size_t i = 0; i < bf::kMotorCount; ++i) {
      motor_commands[i] = commands.command[i];
    }

    const VehicleState& before = vehicle.state();
    const double height_m = -before.body.position_ned.z();
    const double climb_mps = -before.body.velocity_ned.z();
    const env::Air air = env::air_at_height(session.atmosphere, height_m);

    if (!paused) {
      if (step_index % rc_every == 0) {
        rc = autopilot.channels(to_seconds(t), height_m, climb_mps, any_motor_running(commands),
                                static_cast<double>(rc_every) * dt_s);
        link.send_rc(to_seconds(t), rc);
      }
      const physics::RigidBodyState state_before = before.body;
      result = vehicle.step(motor_commands, air.density_kg_m3, dt_s);
      if (step_index % fdm_every == 0) {
        last_fdm = bf::FdmInput{.sim_time_s = to_seconds(t),
                                .angular_rate_frd = result.imu.angular_rate_frd,
                                .specific_force_frd = result.imu.specific_force_frd,
                                .q_ned_from_frd = state_before.q_ned_from_frd,
                                .velocity_ned = state_before.velocity_ned,
                                .longitude_rad = session.origin.longitude_rad,
                                .latitude_rad = session.origin.latitude_rad,
                                .altitude_m = session.origin.altitude_m + height_m,
                                .pressure_pa = air.pressure_pa};
        have_fdm = true;
        link.send_fdm(last_fdm);
      }
      t += step;
      ++step_index;
    } else if (wall_tick % fdm_every == 0 && have_fdm) {
      // Keep Betaflight fed with the frozen state so it neither times out nor loses RC
      link.send_fdm(last_fdm);
      if (wall_tick % rc_every == 0) {
        link.send_rc(to_seconds(t), rc);
      }
    }

    if (!snapshots.try_push(make_snapshot(t, step_index, vehicle, result, commands, rc, paused,
                                          air.density_kg_m3, counters, link.counters()))) {
      ++dropped_snapshots;
    }

    ++wall_tick;
    const auto step_end = std::chrono::steady_clock::now();
    counters.stats.add(std::chrono::duration<double, std::micro>(step_end - step_begin).count());
    const auto deadline = wall_start + std::chrono::nanoseconds(wall_tick * step.ns);
    if (step_end > deadline + std::chrono::nanoseconds(step.ns)) {
      ++counters.overruns;
    }
    std::this_thread::sleep_until(deadline);
  }

  const io::IoStats io_stats = io.stop();
  return RunSummary{.steps = step_index,
                    .overruns = counters.overruns,
                    .motor_packets = link.counters().motor_packets,
                    .malformed_packets = link.counters().malformed_packets,
                    .dropped_log_rows = dropped_snapshots,
                    .render_states_sent = io_stats.render_states_sent,
                    .api_requests = io_stats.api_requests,
                    .final_height_m = -vehicle.state().body.position_ned.z(),
                    .crashed = vehicle.state().crashed};
}

}  // namespace fpvsim::sim
