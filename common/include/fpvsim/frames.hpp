#pragma once

#include <algorithm>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace fpvsim::frames {

// x_g = E, y_g = -D, z_g = -N
inline Eigen::Matrix3d R_godot_from_ned() {
  Eigen::Matrix3d m;
  m << 0.0, 1.0, 0.0,  //
      0.0, 0.0, -1.0,  //
      -1.0, 0.0, 0.0;
  return m;
}

// x_g = R, y_g = -D, z_g = -F
inline Eigen::Matrix3d R_godotbody_from_frd() { return R_godot_from_ned(); }

// x_gltf = -R, y_gltf = -D, z_gltf = F
inline Eigen::Matrix3d R_gltf_from_frd() {
  Eigen::Matrix3d m;
  m << 0.0, -1.0, 0.0,  //
      0.0, 0.0, -1.0,   //
      1.0, 0.0, 0.0;
  return m;
}

inline Eigen::Vector3d godot_from_ned(const Eigen::Vector3d& v_ned) {
  return R_godot_from_ned() * v_ned;
}

inline Eigen::Vector3d godotbody_from_frd(const Eigen::Vector3d& v_frd) {
  return R_godotbody_from_frd() * v_frd;
}

inline Eigen::Vector3d gltf_from_frd(const Eigen::Vector3d& v_frd) {
  return R_gltf_from_frd() * v_frd;
}

// R_godot = C * R_ned * C^T with C a proper rotation: w is kept, the vector part is mapped by C
inline Eigen::Quaterniond q_godot_from_godotbody(const Eigen::Quaterniond& q_ned_from_frd) {
  const Eigen::Vector3d v = R_godot_from_ned() * q_ned_from_frd.vec();
  return {q_ned_from_frd.w(), v.x(), v.y(), v.z()};
}

struct EulerZyx {
  double roll_rad;
  double pitch_rad;
  double yaw_rad;
};

// Aerospace roll, pitch, yaw of q_ned_from_frd (yaw from North, positive to the East)
inline EulerZyx euler_zyx_from_q(const Eigen::Quaterniond& q) {
  const double w = q.w();
  const double x = q.x();
  const double y = q.y();
  const double z = q.z();
  const double sin_pitch = std::clamp(2.0 * (w * y - z * x), -1.0, 1.0);
  return {.roll_rad = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)),
          .pitch_rad = std::asin(sin_pitch),
          .yaw_rad = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))};
}

}  // namespace fpvsim::frames
