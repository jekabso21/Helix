#include <fpvsim/sim/loop.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

#include <fpvsim/bridge/betaflight.hpp>
#include <fpvsim/env/atmosphere.hpp>
#include <fpvsim/log/csv_log.hpp>
#include <fpvsim/pilot/altitude_hold.hpp>
#include <fpvsim/sim/vehicle.hpp>
#include <fpvsim/sim_time.hpp>

namespace fpvsim::sim {

namespace {

namespace bf = bridge::betaflight;

constexpr std::int64_t kFdmRateHz = 1000;
constexpr double kArmedCommandThreshold = 0.01;

bool any_motor_running(const bf::MotorCommands& commands) {
  return std::ranges::any_of(commands.command, [](double c) { return c > kArmedCommandThreshold; });
}

log::TruthRow make_row(SimTime t, const VehicleState& state, const bf::MotorCommands& commands,
                       const bf::RcChannels& rc) {
  const auto& b = state.body;
  return log::TruthRow{
      .sim_time_ns = t.ns,
      .position_ned = {b.position_ned.x(), b.position_ned.y(), b.position_ned.z()},
      .velocity_ned = {b.velocity_ned.x(), b.velocity_ned.y(), b.velocity_ned.z()},
      .q_ned_from_frd = {b.q_ned_from_frd.w(), b.q_ned_from_frd.x(), b.q_ned_from_frd.y(),
                         b.q_ned_from_frd.z()},
      .angular_rate_frd = {b.angular_rate_frd.x(), b.angular_rate_frd.y(), b.angular_rate_frd.z()},
      .motor_command = commands.command,
      .rc_aetr = {rc[0], rc[1], rc[2], rc[3]},
      .crashed = state.crashed};
}

}  // namespace

RunSummary run_realtime(const config::SessionConfig& session, const VehicleParams& drone) {
  const SimTime step = step_from_rate_hz(session.physics_rate_hz).value_or(SimTime{});
  const double dt_s = to_seconds(step);
  const std::int64_t fdm_every = session.physics_rate_hz / kFdmRateHz;
  const std::int64_t rc_every = session.physics_rate_hz / session.input.rc_rate_hz;
  const std::int64_t log_every = session.physics_rate_hz / session.logging.rate_hz;
  const std::int64_t total_steps =
      session.duration_s > 0.0
          ? static_cast<std::int64_t>(
                std::llround(session.duration_s * static_cast<double>(session.physics_rate_hz)))
          : -1;

  Vehicle vehicle(drone, spawn_state(drone, session.spawn.north_m, session.spawn.east_m,
                                     session.spawn.height_agl_m, session.spawn.heading_rad));
  bf::BetaflightLink link(session.betaflight);
  pilot::AltitudeHoldPilot autopilot(session.input.altitude_hold);
  log::TruthLog truth(session.logging.truth_csv);

  bf::MotorCommands commands{};
  bf::RcChannels rc{};
  rc.fill(1000);
  MotorCommandArray motor_commands{};
  std::uint64_t overruns = 0;
  SimTime t{};
  std::int64_t step_index = 0;

  const auto wall_start = std::chrono::steady_clock::now();
  while (total_steps < 0 || step_index < total_steps) {
    link.poll_motors(commands);
    for (std::size_t i = 0; i < bf::kMotorCount; ++i) {
      motor_commands[i] = commands.command[i];
    }

    const VehicleState& before = vehicle.state();
    const double height_m = -before.body.position_ned.z();
    const double climb_mps = -before.body.velocity_ned.z();
    if (step_index % rc_every == 0) {
      rc = autopilot.channels(to_seconds(t), height_m, climb_mps, any_motor_running(commands),
                              static_cast<double>(rc_every) * dt_s);
      link.send_rc(to_seconds(t), rc);
    }

    const env::Air air = env::air_at_height(session.atmosphere, height_m);
    const StepResult result = vehicle.step(motor_commands, air.density_kg_m3, dt_s);

    if (step_index % fdm_every == 0) {
      link.send_fdm(bf::FdmInput{.sim_time_s = to_seconds(t),
                                 .angular_rate_frd = result.imu.angular_rate_frd,
                                 .specific_force_frd = result.imu.specific_force_frd,
                                 .q_ned_from_frd = before.body.q_ned_from_frd,
                                 .velocity_ned = before.body.velocity_ned,
                                 .longitude_rad = session.origin.longitude_rad,
                                 .latitude_rad = session.origin.latitude_rad,
                                 .altitude_m = session.origin.altitude_m + height_m,
                                 .pressure_pa = air.pressure_pa});
    }
    if (step_index % log_every == 0) {
      truth.push(make_row(t, vehicle.state(), commands, rc));
    }

    t += step;
    ++step_index;
    const auto deadline = wall_start + std::chrono::nanoseconds(t.ns);
    if (std::chrono::steady_clock::now() > deadline + std::chrono::nanoseconds(step.ns)) {
      ++overruns;
    }
    std::this_thread::sleep_until(deadline);
  }

  return RunSummary{.steps = step_index,
                    .overruns = overruns,
                    .motor_packets = link.counters().motor_packets,
                    .malformed_packets = link.counters().malformed_packets,
                    .dropped_log_rows = truth.dropped_rows(),
                    .final_height_m = -vehicle.state().body.position_ned.z(),
                    .crashed = vehicle.state().crashed};
}

}  // namespace fpvsim::sim
