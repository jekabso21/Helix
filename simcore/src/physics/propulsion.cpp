#include <fpvsim/physics/propulsion.hpp>

namespace fpvsim::physics {

Loads propulsion_loads(const std::array<MotorMount, kMaxMotors>& mounts,
                       const std::array<MotorParams, kMaxMotors>& motors,
                       const std::array<MotorOutput, kMaxMotors>& outputs,
                       std::size_t motor_count) {
  Loads loads{.force_frd = Eigen::Vector3d::Zero(),
              .force_ned = Eigen::Vector3d::Zero(),
              .torque_frd = Eigen::Vector3d::Zero(),
              .rotor_momentum_frd = Eigen::Vector3d::Zero()};
  for (std::size_t i = 0; i < motor_count; ++i) {
    const MotorMount& mount = mounts[i];
    const Eigen::Vector3d thrust = outputs[i].thrust_n * mount.axis_frd;
    loads.force_frd += thrust;
    loads.torque_frd += mount.position_frd.cross(thrust) +
                        mount.spin * outputs[i].reaction_torque_nm * mount.axis_frd;
    loads.rotor_momentum_frd +=
        motors[i].rotor_inertia_kg_m2 * outputs[i].speed_radps * mount.spin * -mount.axis_frd;
  }
  return loads;
}

Loads rotor_drag_loads(const std::array<MotorMount, kMaxMotors>& mounts,
                       const std::array<MotorParams, kMaxMotors>& motors,
                       const std::array<MotorOutput, kMaxMotors>& outputs, std::size_t motor_count,
                       const Eigen::Vector3d& air_velocity_frd,
                       const Eigen::Vector3d& angular_rate_frd) {
  Loads loads{.force_frd = Eigen::Vector3d::Zero(),
              .force_ned = Eigen::Vector3d::Zero(),
              .torque_frd = Eigen::Vector3d::Zero(),
              .rotor_momentum_frd = Eigen::Vector3d::Zero()};
  for (std::size_t i = 0; i < motor_count; ++i) {
    const MotorMount& mount = mounts[i];
    const Eigen::Vector3d hub_velocity =
        air_velocity_frd + angular_rate_frd.cross(mount.position_frd);
    const Eigen::Vector3d in_plane =
        hub_velocity - hub_velocity.dot(mount.axis_frd) * mount.axis_frd;
    const Eigen::Vector3d force =
        -motors[i].rotor_drag_coefficient * outputs[i].speed_radps * in_plane;
    loads.force_frd += force;
    loads.torque_frd += mount.position_frd.cross(force);
  }
  return loads;
}

}  // namespace fpvsim::physics
