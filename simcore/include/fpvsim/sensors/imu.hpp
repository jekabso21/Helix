#pragma once

#include <Eigen/Core>

#include <fpvsim/physics/rigid_body.hpp>

namespace fpvsim::sensors {

struct ImuSample {
  Eigen::Vector3d angular_rate_frd;
  Eigen::Vector3d specific_force_frd;  // (0, 0, -g) at rest and level
};

// Ideal IMU at imu_offset_frd from the CG, physically correct in FRD
ImuSample ideal_imu(const physics::RigidBodyState& state, const physics::Derivatives& rates,
                    const Eigen::Vector3d& imu_offset_frd);

}  // namespace fpvsim::sensors
