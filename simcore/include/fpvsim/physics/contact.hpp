#pragma once

#include <array>
#include <cstddef>

#include <Eigen/Core>

#include <fpvsim/physics/rigid_body.hpp>

namespace fpvsim::physics {

inline constexpr std::size_t kMaxContactPoints = 16;

struct ContactParams {
  std::array<Eigen::Vector3d, kMaxContactPoints> points_frd;  // relative to the CG
  std::size_t point_count;
  double stiffness_n_per_m;
  double damping_n_s_per_m;
  double friction;
  double friction_regularization_mps;
};

struct ContactLoads {
  Eigen::Vector3d force_ned;
  Eigen::Vector3d torque_frd;
  bool touching;
  double max_closing_speed_mps;  // fastest downward speed of a point that is in contact
};

// Flat ground at ground_down_m in NED; spring-damper normal force and regularized Coulomb friction
ContactLoads contact_loads(const ContactParams& params, const RigidBodyState& state,
                           double ground_down_m);

}  // namespace fpvsim::physics
