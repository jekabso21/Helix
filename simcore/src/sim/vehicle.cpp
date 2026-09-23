#include <fpvsim/sim/vehicle.hpp>

#include <algorithm>
#include <utility>

#include <Eigen/Geometry>

#include <fpvsim/constants.hpp>

namespace fpvsim::sim {

Vehicle::Vehicle(VehicleParams params, const physics::RigidBodyState& spawn)
    : params_(std::move(params)),
      state_{.body = spawn,
             .motor_speed_radps = {},
             .motor_consumed_ah = {},
             .motor_bus_current_a = {},
             .battery = physics::initial_battery(params_.battery),
             .crashed = false},
      imu_noise_(params_.imu_noise) {}

void Vehicle::reset(const physics::RigidBodyState& spawn) {
  state_ = VehicleState{.body = spawn,
                        .motor_speed_radps = {},
                        .motor_consumed_ah = {},
                        .motor_bus_current_a = {},
                        .battery = physics::initial_battery(params_.battery),
                        .crashed = false};
}

void Vehicle::reload(const VehicleParams& params, const physics::RigidBodyState& spawn) {
  params_ = params;
  imu_noise_ = sensors::ImuNoise(params_.imu_noise);
  reset(spawn);
}

double Vehicle::hover_command() const {
  const double thrust_per_motor =
      params_.mass.mass_kg * kStandardGravityMps2 / static_cast<double>(params_.motor_count);
  return physics::hover_command(params_.motors[0], params_.props[0], thrust_per_motor,
                                physics::initial_battery(params_.battery).bus_voltage_v);
}

StepResult Vehicle::step(const MotorCommandArray& commands, const env::Air& air, double time_s,
                         double dt_s) {
  StepResult result{};
  const physics::RigidBodyState& body = state_.body;
  const Eigen::Vector3d air_velocity_frd = body.q_ned_from_frd.conjugate() * body.velocity_ned;
  const bool motors_off = state_.crashed || state_.battery.cutoff;
  // bus voltage and motor currents depend on each other; DC motors are linear in the voltage
  double fixed_current = params_.battery.avionics_current_a;
  double conductance = 0.0;
  for (std::size_t i = 0; i < params_.motor_count; ++i) {
    const physics::MotorParams& motor = params_.motors[i];
    const double u = motors_off ? 0.0 : physics::quantize_command(commands[i]);
    if (motor.model == physics::MotorModel::kDc) {
      const double kt = 1.0 / motor.kv_radps_per_v;
      conductance += u * u / motor.resistance_ohm;
      fixed_current -= u * kt * state_.motor_speed_radps[i] / motor.resistance_ohm;
    } else {
      fixed_current += state_.motor_bus_current_a[i];  // no electrical model: last step's draw
    }
  }
  const double bus_voltage =
      physics::solve_bus_voltage(params_.battery, state_.battery, fixed_current, conductance);
  double bus_current = 0.0;
  for (std::size_t i = 0; i < params_.motor_count; ++i) {
    const physics::MotorMount& mount = params_.mounts[i];
    const Eigen::Vector3d hub_velocity =
        air_velocity_frd + body.angular_rate_frd.cross(mount.position_frd);
    const Eigen::Vector3d hub_ned = body.position_ned + body.q_ned_from_frd * mount.position_frd;
    const physics::MotorInput input{.command = motors_off ? 0.0 : commands[i],
                                    .bus_voltage_v = bus_voltage,
                                    .air_density_kg_m3 = air.density_kg_m3,
                                    .axial_inflow_mps = hub_velocity.dot(mount.axis_frd),
                                    .height_above_ground_m = std::max(-hub_ned.z(), 0.0)};
    result.motors[i] = physics::step_motor(params_.motors[i], params_.props[i],
                                           state_.motor_speed_radps[i], input, dt_s);
    state_.motor_speed_radps[i] = result.motors[i].speed_radps;
    bus_current += result.motors[i].bus_current_a;
    state_.motor_bus_current_a[i] = result.motors[i].bus_current_a;
    state_.motor_consumed_ah[i] += result.motors[i].bus_current_a * dt_s / 3600.0;
  }
  state_.battery = physics::step_battery(params_.battery, state_.battery, bus_current, dt_s);

  physics::Loads loads =
      physics::propulsion_loads(params_.mounts, params_.motors, result.motors, params_.motor_count);
  const physics::AeroLoads aero =
      physics::aero_loads(params_.aero, air.density_kg_m3, air_velocity_frd, body.angular_rate_frd);
  const physics::Loads rotor_drag =
      physics::rotor_drag_loads(params_.mounts, params_.props, result.motors, params_.motor_count,
                                air_velocity_frd, body.angular_rate_frd);
  loads.force_frd += rotor_drag.force_frd;
  loads.torque_frd += rotor_drag.torque_frd;
  const physics::ContactLoads contact = physics::contact_loads(params_.contact, body, 0.0);
  loads.force_frd += aero.force_frd;
  loads.torque_frd += aero.torque_frd + contact.torque_frd;
  loads.force_ned += contact.force_ned;

  result.rates = physics::derivatives(body, params_.mass, loads);
  result.imu_ideal = sensors::ideal_imu(body, result.rates, params_.imu_offset_frd);
  result.imu = imu_noise_.apply(result.imu_ideal, result.motors, params_.motor_count,
                                params_.props[0].blades, time_s);
  result.baro = imu_noise_.apply_baro(air.pressure_pa);
  result.touching = contact.touching;
  if (contact.max_closing_speed_mps > params_.crash_speed_mps) {
    state_.crashed = true;
  }
  state_.body = physics::integrate(body, result.rates, dt_s);
  return result;
}

physics::RigidBodyState spawn_state(const VehicleParams& params, double north_m, double east_m,
                                    double height_agl_m, double heading_rad) {
  double lowest_point_down = 0.0;
  for (std::size_t i = 0; i < params.contact.point_count; ++i) {
    lowest_point_down = std::max(lowest_point_down, params.contact.points_frd[i].z());
  }
  physics::RigidBodyState state = physics::level_state_at(
      Eigen::Vector3d(north_m, east_m, -(height_agl_m + lowest_point_down)));
  state.q_ned_from_frd =
      Eigen::Quaterniond(Eigen::AngleAxisd(heading_rad, Eigen::Vector3d::UnitZ()));
  return state;
}

}  // namespace fpvsim::sim
