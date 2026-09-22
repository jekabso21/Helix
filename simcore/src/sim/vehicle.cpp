#include <fpvsim/sim/vehicle.hpp>

#include <algorithm>
#include <utility>

#include <Eigen/Geometry>

#include <fpvsim/constants.hpp>

namespace fpvsim::sim {

Vehicle::Vehicle(VehicleParams params, const physics::RigidBodyState& spawn)
    : params_(std::move(params)),
      state_{.body = spawn, .motor_speed_radps = {}, .crashed = false} {}

void Vehicle::reset(const physics::RigidBodyState& spawn) {
  state_ = VehicleState{.body = spawn, .motor_speed_radps = {}, .crashed = false};
}

void Vehicle::reload(const VehicleParams& params, const physics::RigidBodyState& spawn) {
  params_ = params;
  reset(spawn);
}

double Vehicle::hover_command() const {
  const double thrust_per_motor =
      params_.mass.mass_kg * kStandardGravityMps2 / static_cast<double>(params_.motor_count);
  return physics::hover_command(params_.motors[0], thrust_per_motor);
}

StepResult Vehicle::step(const MotorCommandArray& commands, double air_density_kg_m3, double dt_s) {
  StepResult result{};
  for (std::size_t i = 0; i < params_.motor_count; ++i) {
    const double command = state_.crashed ? 0.0 : commands[i];
    result.motors[i] =
        physics::step_motor(params_.motors[i], state_.motor_speed_radps[i], command, dt_s);
    state_.motor_speed_radps[i] = result.motors[i].speed_radps;
  }

  const physics::RigidBodyState& body = state_.body;
  physics::Loads loads =
      physics::propulsion_loads(params_.mounts, params_.motors, result.motors, params_.motor_count);
  const Eigen::Vector3d air_velocity_frd = body.q_ned_from_frd.conjugate() * body.velocity_ned;
  const physics::AeroLoads aero =
      physics::aero_loads(params_.aero, air_density_kg_m3, air_velocity_frd, body.angular_rate_frd);
  const physics::Loads rotor_drag =
      physics::rotor_drag_loads(params_.mounts, params_.motors, result.motors, params_.motor_count,
                                air_velocity_frd, body.angular_rate_frd);
  loads.force_frd += rotor_drag.force_frd;
  loads.torque_frd += rotor_drag.torque_frd;
  const physics::ContactLoads contact = physics::contact_loads(params_.contact, body, 0.0);
  loads.force_frd += aero.force_frd;
  loads.torque_frd += aero.torque_frd + contact.torque_frd;
  loads.force_ned += contact.force_ned;

  result.rates = physics::derivatives(body, params_.mass, loads);
  result.imu = sensors::ideal_imu(body, result.rates, params_.imu_offset_frd);
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
