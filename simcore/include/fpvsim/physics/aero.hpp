#pragma once

#include <Eigen/Core>

namespace fpvsim::physics {

struct AeroParams {
  Eigen::Vector3d drag_area_frd_m2;  // Cd * A per body axis
  Eigen::Vector3d center_of_pressure_frd;
  Eigen::Vector3d angular_damping;  // N m per rad/s, per body axis
};

struct AeroLoads {
  Eigen::Vector3d force_frd;
  Eigen::Vector3d torque_frd;
};

// F = -1/2 rho |v| (CdA * v) per axis, with |v| the norm of the whole air-relative velocity
AeroLoads aero_loads(const AeroParams& params, double air_density_kg_m3,
                     const Eigen::Vector3d& air_velocity_frd,
                     const Eigen::Vector3d& angular_rate_frd);

}  // namespace fpvsim::physics
