#include <fpvsim/sim/loop.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <numbers>
#include <string>
#include <thread>

#include <fpvsim/bridge/betaflight.hpp>
#include <fpvsim/env/atmosphere.hpp>
#include <fpvsim/input/control.hpp>
#include <fpvsim/input/joystick.hpp>
#include <fpvsim/io/io_thread.hpp>
#include <fpvsim/pilot/altitude_hold.hpp>
#include <fpvsim/sim/snapshot.hpp>
#include <fpvsim/sim/step_stats.hpp>
#include <fpvsim/sim/vehicle.hpp>
#include <fpvsim/sim_time.hpp>

namespace fpvsim::sim {

namespace {

std::atomic<bool> g_stop_requested{false};

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
                       const LoopCounters& counters, const bf::LinkCounters& link,
                       const input::DeviceState& device, const input::DeviceInfo& device_info) {
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
  for (std::size_t i = 0; i < input::kMaxAxes; ++i) {
    s.raw_axes[i] = static_cast<float>(device.axes[i]);
  }
  for (std::size_t i = 0; i < input::kMaxButtons; ++i) {
    s.raw_buttons[i] = device.buttons[i] ? 1 : 0;
  }
  s.raw_axis_count = static_cast<std::uint8_t>(std::min(device_info.axis_count, input::kMaxAxes));
  s.raw_button_count =
      static_cast<std::uint8_t>(std::min(device_info.button_count, input::kMaxButtons));
  s.device_connected = device_info.connected;
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
                    MotorCommandArray& motor_commands, bool& paused,
                    input::InputControl& input_control, input::JoystickManager* joystick,
                    input::InputMapping& mapping) {
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
      case CommandType::kSetInputMapping:
        input_control.take_mapping(mapping);
        break;
      case CommandType::kSelectInputDevice: {
        std::string name;
        if (joystick != nullptr && input_control.take_device(name)) {
          joystick->open(name);
          mapping.device_name_contains = name;
        }
        break;
      }
    }
    if (joystick != nullptr) {
      input_control.publish_status(
          input::InputStatus{.source = "gamepad",
                             .device = joystick->info(),
                             .mapping = mapping,
                             .device_names = input::JoystickManager::device_names()});
    }
    results_out.try_push(CommandResult{
        .client = command.client, .request_id = command.request_id, .applied_at_ns = t.ns});
  }
  return running;
}

// While paused Betaflight keeps getting the frozen state so it neither times out nor loses RC
void feed_frozen_state(bf::BetaflightLink& link, const bf::FdmInput& last_fdm,
                       const bf::RcChannels& rc, std::int64_t wall_tick, std::int64_t fdm_every,
                       std::int64_t rc_every) {
  if (wall_tick % fdm_every == 0) {
    link.send_fdm(last_fdm);
  }
  if (wall_tick % rc_every == 0) {
    link.send_rc(last_fdm.sim_time_s, rc);
  }
}

bf::RcChannels read_rc(input::JoystickManager* joystick, const input::InputMapping& mapping,
                       input::DeviceState& device, pilot::AltitudeHoldPilot& autopilot,
                       double sim_time_s, double height_m, double climb_mps, bool armed,
                       double dt_s) {
  if (joystick != nullptr) {
    device = joystick->poll();
    return input::map_channels(mapping, device);
  }
  return autopilot.channels(sim_time_s, height_m, climb_mps, armed, dt_s);
}

}  // namespace

void request_stop() noexcept { g_stop_requested.store(true, std::memory_order_relaxed); }

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
  const bool use_joystick = session.input.source == "gamepad";
  input::InputMapping mapping = session.input.mapping;
  std::unique_ptr<input::JoystickManager> joystick;
  input::InputControl input_control;
  if (use_joystick) {
    joystick = std::make_unique<input::JoystickManager>();
    joystick->open(mapping.device_name_contains);
    input_control.publish_status(
        input::InputStatus{.source = session.input.source,
                           .device = joystick->info(),
                           .mapping = mapping,
                           .device_names = input::JoystickManager::device_names()});
  }

  io::SnapshotQueue snapshots;
  io::CommandQueue commands_in;
  io::ResultQueue results_out;
  io::IoThread io(input_control,
                  io::IoConfig{.api_host = session.control_api.host,
                               .api_port = session.control_api.port,
                               .app_host = session.app.host,
                               .app_port = session.app.port,
                               .physics_rate_hz = session.physics_rate_hz,
                               .state_rate_hz = session.app.state_rate_hz,
                               .log_rate_hz = session.logging.rate_hz,
                               .truth_csv = session.logging.truth_csv,
                               .input_source = session.input.source,
                               .info = info},
                  snapshots, commands_in, results_out);

  bf::MotorCommands commands{};
  bf::RcChannels rc{};
  rc.fill(1000);
  input::DeviceState device{};
  input::DeviceInfo device_info = joystick ? joystick->info() : input::DeviceInfo{};
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
  while (running && !g_stop_requested.load(std::memory_order_relaxed) &&
         (total_steps < 0 || step_index < total_steps)) {
    const auto step_begin = std::chrono::steady_clock::now();

    running = apply_commands(commands_in, results_out, t, vehicle, spawn, autopilot, commands,
                             motor_commands, paused, input_control, joystick.get(), mapping);

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
        rc = read_rc(joystick.get(), mapping, device, autopilot, to_seconds(t), height_m, climb_mps,
                     any_motor_running(commands), static_cast<double>(rc_every) * dt_s);
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
    } else if (have_fdm) {
      feed_frozen_state(link, last_fdm, rc, wall_tick, fdm_every, rc_every);
    }

    if (joystick && step_index % session.physics_rate_hz == 0) {
      device_info = joystick->info();
    }
    if (!snapshots.try_push(make_snapshot(t, step_index, vehicle, result, commands, rc, paused,
                                          air.density_kg_m3, counters, link.counters(), device,
                                          device_info))) {
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
