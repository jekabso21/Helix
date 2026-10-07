#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <numbers>

#include <fpvsim/constants.hpp>
#include <fpvsim/sim/vehicle.hpp>

#include "test_quad.hpp"

namespace sim = fpvsim::sim;
namespace quad = fpvsim::testing;
using Eigen::Vector3d;

namespace {

sim::VehicleParams quad_params() {
  return sim::VehicleParams{.mass = quad::quad_mass(),
                            .motor_count = quad::kMotorCount,
                            .mounts = quad::quad_mounts(),
                            .motors = quad::quad_motors(),
                            .props = quad::quad_props(),
                            .battery = quad::quad_battery(),
                            .aero = {.drag_area_frd_m2 = Vector3d(0.01, 0.01, 0.02),
                                     .center_of_pressure_frd = Vector3d::Zero(),
                                     .angular_damping = Vector3d(0.0005, 0.0005, 0.0005)},
                            .contact = quad::quad_contact(),
                            .crash_speed_mps = 6.0,
                            .imu_offset_frd = Vector3d::Zero(),
                            .imu_noise = quad::quiet_imu()};
}

const fpvsim::env::Air kAir{
    .temperature_k = 288.15, .pressure_pa = 101325.0, .density_kg_m3 = 1.225};

}  // namespace

TEST(VehicleTest, SpawnRestsOnTheGroundWithoutSinkingOrBouncing) {
  const sim::VehicleParams params = quad_params();
  sim::Vehicle vehicle(params, sim::spawn_state(params, 0.0, 0.0, 0.0, 0.0));
  const double start_z = vehicle.state().body.position_ned.z();
  for (int i = 0; i < 3000; ++i) {
    vehicle.step({}, kAir, 0.0, 0.001);
  }
  EXPECT_NEAR(vehicle.state().body.position_ned.z(), start_z, 0.002);
  EXPECT_FALSE(vehicle.state().crashed);
  EXPECT_LT(vehicle.state().body.velocity_ned.norm(), 1e-4);
}

TEST(VehicleTest, HoverCommandReachesEquilibriumInStillAir) {
  sim::VehicleParams params = quad_params();
  // hover_command() assumes the open-circuit voltage; no sag keeps the balance exact
  params.battery.cell_resistance_ohm = 0.0;
  params.battery.connector_resistance_ohm = 0.0;
  sim::Vehicle vehicle(params, sim::spawn_state(params, 0.0, 0.0, 10.0, 0.0));
  sim::MotorCommandArray commands{};
  commands.fill(vehicle.hover_command());
  const double start_z = vehicle.state().body.position_ned.z();
  sim::StepResult result{};
  for (int i = 0; i < 5000; ++i) {
    result = vehicle.step(commands, kAir, 0.0, 0.001);
  }
  // Motors spin up from rest, so the drone sinks a little before thrust balances weight
  EXPECT_GT(vehicle.state().body.position_ned.z(), start_z);
  EXPECT_LT(vehicle.state().body.position_ned.z(), start_z + 3.0);
  // Left over: drag on the residual sink velocity and the DShot step of the command (0.1 %)
  EXPECT_NEAR(result.rates.acceleration_ned.z(), 0.0, 3e-2);
  EXPECT_NEAR(result.imu.specific_force_frd.z(), -fpvsim::kStandardGravityMps2, 3e-2);
  EXPECT_LT(vehicle.state().body.angular_rate_frd.norm(), 1e-6);
  EXPECT_FALSE(result.touching);
}

TEST(VehicleTest, FallingFromHeightCrashesAndCutsMotors) {
  const sim::VehicleParams params = quad_params();
  sim::Vehicle vehicle(params, sim::spawn_state(params, 0.0, 0.0, 5.0, 0.0));
  sim::MotorCommandArray full{};
  full.fill(1.0);
  bool crashed = false;
  for (int i = 0; i < 3000 && !crashed; ++i) {
    vehicle.step({}, kAir, 0.0, 0.001);
    crashed = vehicle.state().crashed;
  }
  ASSERT_TRUE(crashed);
  for (int i = 0; i < 1000; ++i) {
    vehicle.step(full, kAir, 0.0, 0.001);
  }
  EXPECT_EQ(vehicle.state().motor_speed_radps[0], 0.0);
  vehicle.reset(sim::spawn_state(params, 0.0, 0.0, 0.0, 0.0));
  EXPECT_FALSE(vehicle.state().crashed);
}

TEST(VehicleTest, HeadingIsAppliedAtSpawn) {
  const sim::VehicleParams params = quad_params();
  const auto state = sim::spawn_state(params, 1.0, 2.0, 0.0, std::numbers::pi / 2.0);
  const Vector3d nose = state.q_ned_from_frd * Vector3d::UnitX();
  EXPECT_NEAR(nose.y(), 1.0, 1e-9);
  EXPECT_DOUBLE_EQ(state.position_ned.x(), 1.0);
  EXPECT_DOUBLE_EQ(state.position_ned.y(), 2.0);
}

TEST(VehicleTest, ReloadSwapsTheModelAndResetsToSpawn) {
  const sim::VehicleParams params = quad_params();
  sim::Vehicle vehicle(params, sim::spawn_state(params, 0.0, 0.0, 5.0, 0.0));
  sim::MotorCommandArray full{};
  full.fill(1.0);
  for (int i = 0; i < 200; ++i) {
    vehicle.step(full, kAir, 0.0, 0.001);
  }
  sim::VehicleParams heavier = params;
  heavier.mass =
      fpvsim::physics::make_mass_properties(2.0 * quad::kMassKg, params.mass.inertia_frd);
  const auto spawn = sim::spawn_state(heavier, 1.0, 2.0, 0.0, 0.0);
  vehicle.reload(heavier, spawn);
  EXPECT_DOUBLE_EQ(vehicle.params().mass.mass_kg, 2.0 * quad::kMassKg);
  EXPECT_NEAR(vehicle.state().body.position_ned.x(), 1.0, 1e-12);
  EXPECT_DOUBLE_EQ(vehicle.state().motor_speed_radps[0], 0.0);
  EXPECT_NEAR(vehicle.hover_command(), std::sqrt(2.0) * sim::Vehicle(params, spawn).hover_command(),
              1e-9);
}

// A CG ahead of the motor centre with equal thrust everywhere pitches the nose up (FRD -y torque)
TEST(VehicleTest, EqualThrustWithAForwardCgPitchesNoseUpByTheMomentBalance) {
  sim::VehicleParams params = quad_params();
  const double shift = 0.010;  // CG 10 mm ahead: every motor moves 10 mm aft relative to it
  for (std::size_t i = 0; i < quad::kMotorCount; ++i) {
    params.mounts[i].position_frd.x() -= shift;
  }
  sim::Vehicle vehicle(params, sim::spawn_state(params, 0.0, 0.0, 5.0, 0.0));
  sim::MotorCommandArray half{};
  half.fill(0.5);
  // first step: the body has no angular rate yet, so no damping or gyroscopic terms
  const sim::StepResult result = vehicle.step(half, kAir, 0.0, 0.001);
  double total_thrust = 0.0;
  for (std::size_t i = 0; i < quad::kMotorCount; ++i) {
    total_thrust += result.motors[i].thrust_n;
  }
  // moment about y of the four upward thrusts at x = -shift: sum(r x F)_y = -shift * T_total
  const double expected_pitch_accel = -shift * total_thrust / params.mass.inertia_frd(1, 1);
  EXPECT_LT(expected_pitch_accel, 0.0);
  EXPECT_NEAR(result.rates.angular_acceleration_frd.y(), expected_pitch_accel,
              1e-6 * std::abs(expected_pitch_accel));
}

// Full throttle from hover on the DC motor: the pack sags but voltage and current stay steady
TEST(VehicleTest, PunchOutSagsWithoutOscillatingOrBrowningOut) {
  sim::VehicleParams params = quad_params();
  params.motors.fill(quad::dc_motor());
  params.battery.cell_resistance_ohm = 0.012;  // a tired pack, the worst case
  params.battery.esc_cutoff_v = 8.0;
  sim::Vehicle vehicle(params, sim::spawn_state(params, 0.0, 0.0, 10.0, 0.0));
  sim::MotorCommandArray full{};
  full.fill(1.0);
  double previous_voltage = vehicle.state().battery.bus_voltage_v;
  double min_voltage = previous_voltage;
  int reversals = 0;
  double last_delta = 0.0;
  for (int i = 0; i < 1500; ++i) {
    vehicle.step(full, kAir, i * 0.001, 0.001);
    const double v = vehicle.state().battery.bus_voltage_v;
    const double delta = v - previous_voltage;
    if (i > 5 && delta * last_delta < 0.0 && std::abs(delta) > 0.01) {
      ++reversals;
    }
    last_delta = delta;
    previous_voltage = v;
    min_voltage = std::min(min_voltage, v);
    EXPECT_FALSE(vehicle.state().battery.cutoff) << "brownout at step " << i;
  }
  EXPECT_LT(min_voltage, 25.0);  // it does sag
  EXPECT_GT(min_voltage, 10.0);  // but stays usable
  EXPECT_LE(reversals, 2);       // no millisecond oscillation
  const auto& b = vehicle.state().battery;
  const double r =
      6.0 * params.battery.cell_resistance_ohm + params.battery.connector_resistance_ohm;
  EXPECT_NEAR(b.bus_voltage_v,
              6.0 * fpvsim::physics::open_circuit_voltage(params.battery, b.soc) - r * b.current_a,
              1e-6);
  EXPECT_GT(b.current_a, 30.0);
}

namespace {

// Terminal speed of a nose-down dive at full throttle; inertia is raised so attitude stays fixed
// and the run measures translation only, with no controller in the loop.
double terminal_speed_mps(bool with_inflow) {
  sim::VehicleParams params = quad_params();
  params.motors.fill(quad::dc_motor());
  for (std::size_t i = 0; i < quad::kMotorCount; ++i) {
    params.props[i].inflow_coefficient = with_inflow ? 1.0 : 0.0;
  }
  params.mass = fpvsim::physics::make_mass_properties(
      quad::kMassKg, (Eigen::Vector3d(1.0, 1.0, 1.0) * 1e4).asDiagonal());
  const double pitch = -30.0 * std::numbers::pi / 180.0;  // nose down
  fpvsim::physics::RigidBodyState spawn = sim::spawn_state(params, 0.0, 0.0, 500.0, 0.0);
  spawn.q_ned_from_frd = Eigen::Quaterniond(Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()));
  sim::Vehicle vehicle(params, spawn);
  sim::MotorCommandArray full{};
  full.fill(1.0);
  double speed = 0.0;
  for (int i = 0; i < 20000; ++i) {  // 20 s is well past convergence
    vehicle.step(full, kAir, i * 0.001, 0.001);
    speed = vehicle.state().body.velocity_ned.head<2>().norm();
  }
  return speed;
}

}  // namespace

TEST(VehicleTest, InflowLossLowersTopSpeed) {
  const double without = terminal_speed_mps(false);
  const double with = terminal_speed_mps(true);
  RecordProperty("top_speed_no_inflow_mps", std::to_string(without));
  RecordProperty("top_speed_with_inflow_mps", std::to_string(with));
  std::printf("top speed: %.1f m/s without inflow, %.1f m/s with (%.0f %% lower)\n", without, with,
              100.0 * (without - with) / without);
  EXPECT_GT(without, 10.0);
  EXPECT_LT(with, without);
  EXPECT_GT(with, 0.5 * without);  // a plausible loss, not a collapse
}

// Air-relative velocity drives the drag: a body at rest in a crosswind is pushed downwind with
// exactly the drag force of the wind speed
TEST(VehicleTest, WindPushesAStillBodyDownwindWithTheDragForce) {
  const sim::VehicleParams params = quad_params();
  sim::Vehicle vehicle(params, sim::spawn_state(params, 0.0, 0.0, 20.0, 0.0));
  const Vector3d wind(0.0, 8.0, 0.0);  // blowing east
  const sim::StepResult result = vehicle.step({}, kAir, wind, 0.0, 0.001);
  const double drag_n = 0.5 * kAir.density_kg_m3 * params.aero.drag_area_frd_m2.y() * 8.0 * 8.0;
  const double expected = drag_n / params.mass.mass_kg;
  EXPECT_NEAR(result.rates.acceleration_ned.y(), expected, 1e-6 * expected + 1e-9);
  EXPECT_NEAR(result.rates.acceleration_ned.x(), 0.0, 1e-9);
  // moving with the air there is no drag at all
  sim::Vehicle drifting(params, sim::spawn_state(params, 0.0, 0.0, 20.0, 0.0));
  fpvsim::physics::RigidBodyState moving = drifting.state().body;
  moving.velocity_ned = wind;
  drifting.reset(moving);
  const sim::StepResult carried = drifting.step({}, kAir, wind, 0.0, 0.001);
  EXPECT_NEAR(carried.rates.acceleration_ned.y(), 0.0, 1e-9);
}

namespace {

sim::FailureModifiers one_motor(std::size_t motor, double gain, double thrust_scale) {
  sim::FailureModifiers m = sim::healthy_modifiers();
  m.motors[motor].output_gain = gain;
  m.motors[motor].thrust_scale = thrust_scale;
  m.motors[motor].torque_scale = thrust_scale;
  return m;
}

// Runs the drone at a fixed command for a while and returns the last step
sim::StepResult spin_up(sim::Vehicle& vehicle, double command, double seconds) {
  sim::MotorCommandArray commands{};
  commands.fill(command);
  sim::StepResult result{};
  for (int i = 0; i < static_cast<int>(seconds * 1000.0); ++i) {
    result = vehicle.step(commands, kAir, i * 0.001, 0.001);
  }
  return result;
}

}  // namespace

TEST(VehicleFailureTest, AMotorThatIsOutMakesNoThrust) {
  const sim::VehicleParams params = quad_params();
  sim::Vehicle vehicle(params, sim::spawn_state(params, 0.0, 0.0, 30.0, 0.0));
  vehicle.set_modifiers(one_motor(1, 0.0, 1.0));
  const sim::StepResult result = spin_up(vehicle, 0.5, 0.3);
  EXPECT_EQ(result.motors[1].thrust_n, 0.0);
  EXPECT_GT(result.motors[0].thrust_n, 1.0);
  // the other three lift a corner and the drone starts to roll or pitch
  EXPECT_GT(vehicle.state().body.angular_rate_frd.head<2>().norm(), 0.1);
}

TEST(VehicleFailureTest, ADamagedPropMakesLessThrustAtTheSameSpeed) {
  const sim::VehicleParams params = quad_params();
  sim::Vehicle healthy(params, sim::spawn_state(params, 0.0, 0.0, 30.0, 0.0));
  sim::Vehicle damaged(params, sim::spawn_state(params, 0.0, 0.0, 30.0, 0.0));
  damaged.set_modifiers(one_motor(2, 1.0, 0.7));
  const sim::StepResult a = spin_up(healthy, 0.5, 0.01);
  const sim::StepResult b = spin_up(damaged, 0.5, 0.01);
  // thrust per (rad/s)^2 drops by the damage at whatever speed the rotor reaches
  const double per_speed_a = a.motors[2].thrust_n / (a.motors[2].speed_radps * a.motors[2].speed_radps);
  const double per_speed_b = b.motors[2].thrust_n / (b.motors[2].speed_radps * b.motors[2].speed_radps);
  EXPECT_NEAR(per_speed_b / per_speed_a, 0.7, 1e-6);
  // the other motors only see the slightly smaller sag of a lighter-loaded battery
  EXPECT_NEAR(b.motors[0].thrust_n, a.motors[0].thrust_n, 1e-3 * a.motors[0].thrust_n);
}

TEST(VehicleFailureTest, ADesyncingEscStopsDrivingDuringItsDropouts) {
  const sim::VehicleParams params = quad_params();
  sim::Vehicle vehicle(params, sim::spawn_state(params, 0.0, 0.0, 30.0, 0.0));
  sim::FailureModifiers m = sim::healthy_modifiers();
  m.motors[0].desync_period_s = 0.2;
  m.motors[0].desync_dropout_s = 0.1;
  vehicle.set_modifiers(m);
  sim::MotorCommandArray commands{};
  commands.fill(0.5);
  double driven = 0.0;
  double dropped = 0.0;
  for (int i = 0; i < 1000; ++i) {
    const double t = i * 0.001;
    const sim::StepResult r = vehicle.step(commands, kAir, t, 0.001);
    (std::fmod(t, 0.2) < 0.1 ? dropped : driven) += r.motors[0].bus_current_a;
  }
  // a freewheeling ESC draws nothing from the battery
  EXPECT_LT(dropped, 0.05 * driven);
}

TEST(VehicleFailureTest, AWeakPackSagsLowerUnderTheSameLoad) {
  const sim::VehicleParams params = quad_params();
  sim::Vehicle healthy(params, sim::spawn_state(params, 0.0, 0.0, 30.0, 0.0));
  sim::Vehicle weak(params, sim::spawn_state(params, 0.0, 0.0, 30.0, 0.0));
  sim::FailureModifiers m = sim::healthy_modifiers();
  m.weak_cell_drop_v = 0.8;
  m.cell_resistance_scale = 3.0;
  weak.set_modifiers(m);
  (void)spin_up(healthy, 0.6, 0.2);
  (void)spin_up(weak, 0.6, 0.2);
  EXPECT_LT(weak.state().battery.bus_voltage_v, healthy.state().battery.bus_voltage_v - 0.8);
}

TEST(VehicleFailureTest, HealthyModifiersChangeNothing) {
  const sim::VehicleParams params = quad_params();
  sim::Vehicle plain(params, sim::spawn_state(params, 0.0, 0.0, 30.0, 0.0));
  sim::Vehicle healthy(params, sim::spawn_state(params, 0.0, 0.0, 30.0, 0.0));
  healthy.set_modifiers(sim::healthy_modifiers());
  (void)spin_up(plain, 0.55, 0.5);
  (void)spin_up(healthy, 0.55, 0.5);
  EXPECT_EQ(plain.state().body.position_ned, healthy.state().body.position_ned);
  EXPECT_EQ(plain.state().battery.bus_voltage_v, healthy.state().battery.bus_voltage_v);
}
