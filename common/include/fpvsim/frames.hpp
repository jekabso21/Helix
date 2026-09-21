#pragma once

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

}  // namespace fpvsim::frames
