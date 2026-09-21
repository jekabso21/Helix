#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

#include <fpvsim/constants.hpp>
#include <fpvsim/sensors/imu.hpp>

namespace physics = fpvsim::physics;
namespace sensors = fpvsim::sensors;
using Eigen::Vector3d;

namespace {
physics::Derivatives rates(const Vector3d& acceleration_ned, const Vector3d& angular_acceleration) {
  return physics::Derivatives{.acceleration_ned = acceleration_ned,
                              .angular_acceleration_frd = angular_acceleration};
}

physics::Derivatives at_rest() { return rates(Vector3d::Zero(), Vector3d::Zero()); }
}  // namespace

TEST(ImuTest, AtRestAndLevelReadsMinusGOnZ) {
  const sensors::ImuSample sample =
      sensors::ideal_imu(physics::level_state_at(Vector3d::Zero()), at_rest(), Vector3d::Zero());
  EXPECT_LT((sample.specific_force_frd - Vector3d(0.0, 0.0, -fpvsim::kStandardGravityMps2)).norm(),
            1e-12);
  EXPECT_EQ(sample.angular_rate_frd.norm(), 0.0);
}

TEST(ImuTest, FreeFallReadsZero) {
  const physics::Derivatives falling =
      rates(Vector3d(0.0, 0.0, fpvsim::kStandardGravityMps2), Vector3d::Zero());
  const sensors::ImuSample sample =
      sensors::ideal_imu(physics::level_state_at(Vector3d::Zero()), falling, Vector3d::Zero());
  EXPECT_LT(sample.specific_force_frd.norm(), 1e-12);
}

TEST(ImuTest, NoseUpAtRestReadsPositiveXAndRollRightReadsNegativeY) {
  const double angle = 20.0 * std::numbers::pi / 180.0;
  const double g = fpvsim::kStandardGravityMps2;

  physics::RigidBodyState nose_up = physics::level_state_at(Vector3d::Zero());
  nose_up.q_ned_from_frd = Eigen::Quaterniond(Eigen::AngleAxisd(angle, Vector3d::UnitY()));
  const Vector3d pitch_force =
      sensors::ideal_imu(nose_up, at_rest(), Vector3d::Zero()).specific_force_frd;
  EXPECT_LT((pitch_force - Vector3d(g * std::sin(angle), 0.0, -g * std::cos(angle))).norm(), 1e-12);

  physics::RigidBodyState roll_right = physics::level_state_at(Vector3d::Zero());
  roll_right.q_ned_from_frd = Eigen::Quaterniond(Eigen::AngleAxisd(angle, Vector3d::UnitX()));
  const Vector3d roll_force =
      sensors::ideal_imu(roll_right, at_rest(), Vector3d::Zero()).specific_force_frd;
  EXPECT_LT((roll_force - Vector3d(0.0, -g * std::sin(angle), -g * std::cos(angle))).norm(), 1e-12);
}

TEST(ImuTest, LeverArmAddsCentripetalAndTangentialTerms) {
  physics::RigidBodyState spinning = physics::level_state_at(Vector3d::Zero());
  spinning.angular_rate_frd = Vector3d(0.0, 0.0, 10.0);
  const Vector3d offset(0.05, 0.0, 0.0);
  const physics::Derivatives hovering = rates(Vector3d::Zero(), Vector3d(0.0, 0.0, 4.0));
  const Vector3d force = sensors::ideal_imu(spinning, hovering, offset).specific_force_frd;
  EXPECT_NEAR(force.x(), -10.0 * 10.0 * 0.05, 1e-12);
  EXPECT_NEAR(force.y(), 4.0 * 0.05, 1e-12);
  EXPECT_NEAR(force.z(), -fpvsim::kStandardGravityMps2, 1e-12);
}
