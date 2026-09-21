#include <fpvsim/physics/propulsion.hpp>

namespace fpvsim::physics {

Loads propulsion_loads(const std::array<MotorMount, kMaxMotors>& mounts,
                       const std::array<MotorOutput, kMaxMotors>& outputs, std::size_t motor_count,
                       double rotor_inertia_kg_m2) {
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
        rotor_inertia_kg_m2 * outputs[i].speed_radps * mount.spin * -mount.axis_frd;
  }
  return loads;
}

}  // namespace fpvsim::physics
