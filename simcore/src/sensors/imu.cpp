#include <fpvsim/sensors/imu.hpp>

#include <fpvsim/constants.hpp>

namespace fpvsim::sensors {

ImuSample ideal_imu(const physics::RigidBodyState& state, const physics::Derivatives& rates,
                    const Eigen::Vector3d& imu_offset_frd) {
  const Eigen::Vector3d gravity_ned(0.0, 0.0, kStandardGravityMps2);
  const Eigen::Vector3d& w = state.angular_rate_frd;
  const Eigen::Vector3d at_cg =
      state.q_ned_from_frd.conjugate() * (rates.acceleration_ned - gravity_ned);
  const Eigen::Vector3d lever_arm =
      rates.angular_acceleration_frd.cross(imu_offset_frd) + w.cross(w.cross(imu_offset_frd));
  return ImuSample{.angular_rate_frd = w, .specific_force_frd = at_cg + lever_arm};
}

}  // namespace fpvsim::sensors
