#include <fpvsim/physics/aero.hpp>

#include <Eigen/Geometry>

namespace fpvsim::physics {

AeroLoads aero_loads(const AeroParams& params, double air_density_kg_m3,
                     const Eigen::Vector3d& air_velocity_frd,
                     const Eigen::Vector3d& angular_rate_frd) {
  const Eigen::Vector3d drag = -0.5 * air_density_kg_m3 * air_velocity_frd.norm() *
                               params.drag_area_frd_m2.cwiseProduct(air_velocity_frd);
  const Eigen::Vector3d damping = -params.angular_damping.cwiseProduct(angular_rate_frd);
  return AeroLoads{.force_frd = drag,
                   .torque_frd = params.center_of_pressure_frd.cross(drag) + damping};
}

}  // namespace fpvsim::physics
