#include <fpvsim/physics/contact.hpp>

#include <algorithm>

namespace fpvsim::physics {

ContactLoads contact_loads(const ContactParams& params, const RigidBodyState& state,
                           double ground_down_m) {
  ContactLoads loads{.force_ned = Eigen::Vector3d::Zero(),
                     .torque_frd = Eigen::Vector3d::Zero(),
                     .touching = false,
                     .max_closing_speed_mps = 0.0};
  const Eigen::Matrix3d r_ned_from_frd = state.q_ned_from_frd.toRotationMatrix();
  for (std::size_t i = 0; i < params.point_count; ++i) {
    const Eigen::Vector3d& point_frd = params.points_frd[i];
    const Eigen::Vector3d position_ned = state.position_ned + r_ned_from_frd * point_frd;
    const double penetration = position_ned.z() - ground_down_m;
    if (penetration <= 0.0) {
      continue;
    }
    const Eigen::Vector3d velocity_ned =
        state.velocity_ned + r_ned_from_frd * state.angular_rate_frd.cross(point_frd);
    const double closing_speed = velocity_ned.z();
    const double normal = std::max(
        0.0, params.stiffness_n_per_m * penetration + params.damping_n_s_per_m * closing_speed);
    const Eigen::Vector3d tangential(velocity_ned.x(), velocity_ned.y(), 0.0);
    const Eigen::Vector3d friction = -params.friction * normal * tangential /
                                     (tangential.norm() + params.friction_regularization_mps);
    const Eigen::Vector3d force_ned = friction + Eigen::Vector3d(0.0, 0.0, -normal);

    loads.force_ned += force_ned;
    loads.torque_frd += point_frd.cross(r_ned_from_frd.transpose() * force_ned);
    loads.touching = true;
    loads.max_closing_speed_mps = std::max(loads.max_closing_speed_mps, closing_speed);
  }
  return loads;
}

}  // namespace fpvsim::physics
