#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

#include <fpvsim/frames.hpp>

namespace frames = fpvsim::frames;
using Eigen::Vector3d;

namespace {
constexpr double kTol = 1e-12;

::testing::AssertionResult VecNear(const Vector3d& actual, const Vector3d& expected) {
  if ((actual - expected).norm() < kTol) {
    return ::testing::AssertionSuccess();
  }
  return ::testing::AssertionFailure()
         << "actual (" << actual.transpose() << ") expected (" << expected.transpose() << ")";
}
}  // namespace

TEST(FramesTest, NedAxesMapToGodotWorldAxes) {
  EXPECT_TRUE(VecNear(frames::godot_from_ned({1, 0, 0}), {0, 0, -1}));
  EXPECT_TRUE(VecNear(frames::godot_from_ned({0, 1, 0}), {1, 0, 0}));
  EXPECT_TRUE(VecNear(frames::godot_from_ned({0, 0, 1}), {0, -1, 0}));
}

TEST(FramesTest, FrdAxesMapToGodotBodyAxes) {
  EXPECT_TRUE(VecNear(frames::godotbody_from_frd({1, 0, 0}), {0, 0, -1}));
  EXPECT_TRUE(VecNear(frames::godotbody_from_frd({0, 1, 0}), {1, 0, 0}));
  EXPECT_TRUE(VecNear(frames::godotbody_from_frd({0, 0, 1}), {0, -1, 0}));
}

TEST(FramesTest, FrdAxesMapToGltfAxes) {
  EXPECT_TRUE(VecNear(frames::gltf_from_frd({1, 0, 0}), {0, 0, 1}));
  EXPECT_TRUE(VecNear(frames::gltf_from_frd({0, 1, 0}), {-1, 0, 0}));
  EXPECT_TRUE(VecNear(frames::gltf_from_frd({0, 0, 1}), {0, -1, 0}));
}

TEST(FramesTest, FrameMappingsAreProperRotations) {
  for (const auto& m : {frames::R_godot_from_ned(), frames::R_gltf_from_frd()}) {
    EXPECT_NEAR(m.determinant(), 1.0, kTol);
    EXPECT_LT((m * m.transpose() - Eigen::Matrix3d::Identity()).norm(), kTol);
  }
}

TEST(FramesTest, YawRight90DegPointsGodotNoseEast) {
  const double half = std::numbers::pi / 4.0;
  const Eigen::Quaterniond q_ned_from_frd(std::cos(half), 0.0, 0.0, std::sin(half));
  EXPECT_TRUE(VecNear(q_ned_from_frd * Vector3d(1, 0, 0), {0, 1, 0}));

  const Eigen::Quaterniond q_godot = frames::q_godot_from_godotbody(q_ned_from_frd);
  const Vector3d nose_godotbody(0, 0, -1);
  EXPECT_TRUE(VecNear(q_godot * nose_godotbody, {1, 0, 0}));
}

TEST(FramesTest, QuaternionConversionMatchesMatrixConjugation) {
  const Eigen::Quaterniond q_ned_from_frd = Eigen::Quaterniond(0.8, 0.1, -0.3, 0.5).normalized();
  const Eigen::Matrix3d c = frames::R_godot_from_ned();
  const Eigen::Matrix3d expected = c * q_ned_from_frd.toRotationMatrix() * c.transpose();
  const Eigen::Matrix3d actual = frames::q_godot_from_godotbody(q_ned_from_frd).toRotationMatrix();
  EXPECT_LT((actual - expected).norm(), kTol);
}
