#include <doctest/doctest.h>

#include <cmath>
#include <numbers>

#include <fpvsim/frames.hpp>

namespace frames = fpvsim::frames;
using Eigen::Vector3d;

namespace {
constexpr double kTol = 1e-12;

void check_vec(const Vector3d& actual, const Vector3d& expected) {
  CHECK((actual - expected).norm() < kTol);
}
}  // namespace

TEST_CASE("ned_axes_map_to_godot_world_axes") {
  check_vec(frames::godot_from_ned({1, 0, 0}), {0, 0, -1});
  check_vec(frames::godot_from_ned({0, 1, 0}), {1, 0, 0});
  check_vec(frames::godot_from_ned({0, 0, 1}), {0, -1, 0});
}

TEST_CASE("frd_axes_map_to_godot_body_axes") {
  check_vec(frames::godotbody_from_frd({1, 0, 0}), {0, 0, -1});
  check_vec(frames::godotbody_from_frd({0, 1, 0}), {1, 0, 0});
  check_vec(frames::godotbody_from_frd({0, 0, 1}), {0, -1, 0});
}

TEST_CASE("frd_axes_map_to_gltf_axes") {
  check_vec(frames::gltf_from_frd({1, 0, 0}), {0, 0, 1});
  check_vec(frames::gltf_from_frd({0, 1, 0}), {-1, 0, 0});
  check_vec(frames::gltf_from_frd({0, 0, 1}), {0, -1, 0});
}

TEST_CASE("frame_mappings_are_proper_rotations") {
  for (const auto& m : {frames::R_godot_from_ned(), frames::R_gltf_from_frd()}) {
    CHECK(m.determinant() == doctest::Approx(1.0).epsilon(kTol));
    CHECK((m * m.transpose() - Eigen::Matrix3d::Identity()).norm() < kTol);
  }
}

TEST_CASE("yaw_right_90_deg_points_godot_nose_east") {
  const double half = std::numbers::pi / 4.0;
  const Eigen::Quaterniond q_ned_from_frd(std::cos(half), 0.0, 0.0, std::sin(half));
  check_vec(q_ned_from_frd * Vector3d(1, 0, 0), {0, 1, 0});

  const Eigen::Quaterniond q_godot = frames::q_godot_from_godotbody(q_ned_from_frd);
  const Vector3d nose_godotbody(0, 0, -1);
  check_vec(q_godot * nose_godotbody, {1, 0, 0});
}

TEST_CASE("quaternion_conversion_matches_matrix_conjugation") {
  const Eigen::Quaterniond q_ned_from_frd = Eigen::Quaterniond(0.8, 0.1, -0.3, 0.5).normalized();
  const Eigen::Matrix3d c = frames::R_godot_from_ned();
  const Eigen::Matrix3d expected = c * q_ned_from_frd.toRotationMatrix() * c.transpose();
  const Eigen::Matrix3d actual = frames::q_godot_from_godotbody(q_ned_from_frd).toRotationMatrix();
  CHECK((actual - expected).norm() < kTol);
}
