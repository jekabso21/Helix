#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace fpvsim::physics {

struct RigidBodyState {
  Eigen::Vector3d position_ned;
  Eigen::Vector3d velocity_ned;
  Eigen::Quaterniond q_ned_from_frd;
  Eigen::Vector3d angular_rate_frd;
};

struct MassProperties {
  double mass_kg;
  Eigen::Matrix3d inertia_frd;
  Eigen::Matrix3d inertia_frd_inverse;
};

// All loads except gravity, which integrate() adds itself
struct Loads {
  Eigen::Vector3d force_frd;
  Eigen::Vector3d force_ned;
  Eigen::Vector3d torque_frd;
  Eigen::Vector3d rotor_momentum_frd;
};

struct Derivatives {
  Eigen::Vector3d acceleration_ned;
  Eigen::Vector3d angular_acceleration_frd;
};

MassProperties make_mass_properties(double mass_kg, const Eigen::Matrix3d& inertia_frd);

RigidBodyState level_state_at(const Eigen::Vector3d& position_ned);

Derivatives derivatives(const RigidBodyState& state, const MassProperties& mass,
                        const Loads& loads);

// Semi-implicit Euler: rates first, then position and attitude from the new rates
RigidBodyState integrate(const RigidBodyState& state, const Derivatives& rates, double dt_s);

}  // namespace fpvsim::physics
