#pragma once

#include <array>
#include <cstddef>

#include <Eigen/Core>

#include <fpvsim/physics/motor.hpp>
#include <fpvsim/physics/rigid_body.hpp>

namespace fpvsim::physics {

inline constexpr std::size_t kMaxMotors = 8;

struct MotorMount {
  Eigen::Vector3d position_frd;  // relative to the CG
  Eigen::Vector3d axis_frd;      // unit thrust direction, (0, 0, -1) is up
  double spin;                   // +1 clockwise seen from above, -1 counter-clockwise
};

// Force, torque and rotor angular momentum of all motors; other Loads fields are zero
Loads propulsion_loads(const std::array<MotorMount, kMaxMotors>& mounts,
                       const std::array<MotorOutput, kMaxMotors>& outputs, std::size_t motor_count,
                       double rotor_inertia_kg_m2);

}  // namespace fpvsim::physics
