#include <fpvsim/physics/rigid_body.hpp>

#include <cmath>

#include <fpvsim/constants.hpp>

namespace fpvsim::physics {

MassProperties make_mass_properties(double mass_kg, const Eigen::Matrix3d& inertia_frd) {
  return MassProperties{
      .mass_kg = mass_kg, .inertia_frd = inertia_frd, .inertia_frd_inverse = inertia_frd.inverse()};
}

RigidBodyState level_state_at(const Eigen::Vector3d& position_ned) {
  return RigidBodyState{.position_ned = position_ned,
                        .velocity_ned = Eigen::Vector3d::Zero(),
                        .q_ned_from_frd = Eigen::Quaterniond::Identity(),
                        .angular_rate_frd = Eigen::Vector3d::Zero()};
}

Derivatives derivatives(const RigidBodyState& state, const MassProperties& mass,
                        const Loads& loads) {
  const Eigen::Vector3d gravity_ned(0.0, 0.0, kStandardGravityMps2);
  const Eigen::Vector3d force_ned = state.q_ned_from_frd * loads.force_frd + loads.force_ned;
  const Eigen::Vector3d& w = state.angular_rate_frd;
  const Eigen::Vector3d gyroscopic = w.cross(mass.inertia_frd * w + loads.rotor_momentum_frd);
  return Derivatives{
      .acceleration_ned = force_ned / mass.mass_kg + gravity_ned,
      .angular_acceleration_frd = mass.inertia_frd_inverse * (loads.torque_frd - gyroscopic)};
}

RigidBodyState integrate(const RigidBodyState& state, const Derivatives& rates, double dt_s) {
  RigidBodyState next = state;
  next.velocity_ned += rates.acceleration_ned * dt_s;
  next.angular_rate_frd += rates.angular_acceleration_frd * dt_s;
  next.position_ned += next.velocity_ned * dt_s;

  const Eigen::Vector3d rotation = next.angular_rate_frd * dt_s;
  const double angle = rotation.norm();
  Eigen::Quaterniond delta = Eigen::Quaterniond::Identity();
  if (angle > 0.0) {
    const Eigen::Vector3d axis = rotation / angle;
    delta = Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis));
  }
  next.q_ned_from_frd = (state.q_ned_from_frd * delta).normalized();
  return next;
}

}  // namespace fpvsim::physics
