#include <gtest/gtest.h>

#include <cmath>

#include <fpvsim/physics/aero.hpp>

namespace physics = fpvsim::physics;
using Eigen::Vector3d;

namespace {
constexpr double kRho = 1.225;

physics::AeroParams params(const Vector3d& drag_area, const Vector3d& cop) {
  return physics::AeroParams{.drag_area_frd_m2 = drag_area,
                             .center_of_pressure_frd = cop,
                             .angular_damping = Vector3d(0.001, 0.002, 0.003)};
}
}  // namespace

TEST(AeroTest, SingleAxisDragIsHalfRhoCdAVSquared) {
  const physics::AeroLoads loads = physics::aero_loads(params({0.01, 0.02, 0.03}, Vector3d::Zero()),
                                                       kRho, {20.0, 0.0, 0.0}, Vector3d::Zero());
  EXPECT_NEAR(loads.force_frd.x(), -0.5 * kRho * 0.01 * 400.0, 1e-12);
  EXPECT_NEAR(loads.force_frd.y(), 0.0, 1e-12);
  EXPECT_NEAR(loads.force_frd.z(), 0.0, 1e-12);
}

TEST(AeroTest, EqualDragAreasGiveAntiparallelForceIndependentOfDirection) {
  const physics::AeroParams isotropic = params({0.02, 0.02, 0.02}, Vector3d::Zero());
  const Vector3d along_axis(15.0, 0.0, 0.0);
  const Vector3d diagonal = Vector3d(1.0, 1.0, 1.0).normalized() * 15.0;
  const Vector3d force_axis =
      physics::aero_loads(isotropic, kRho, along_axis, Vector3d::Zero()).force_frd;
  const Vector3d force_diagonal =
      physics::aero_loads(isotropic, kRho, diagonal, Vector3d::Zero()).force_frd;
  EXPECT_NEAR(force_axis.norm(), force_diagonal.norm(), 1e-12);
  EXPECT_LT(force_diagonal.normalized().dot(diagonal.normalized()), -1.0 + 1e-12);
}

TEST(AeroTest, DragAlwaysOpposesAirRelativeVelocity) {
  const physics::AeroParams anisotropic = params({0.005, 0.02, 0.05}, Vector3d::Zero());
  for (const Vector3d& velocity :
       {Vector3d(10.0, -3.0, 2.0), Vector3d(-1.0, 8.0, -6.0), Vector3d(0.1, 0.1, -25.0)}) {
    const Vector3d force =
        physics::aero_loads(anisotropic, kRho, velocity, Vector3d::Zero()).force_frd;
    EXPECT_LT(force.dot(velocity), 0.0);
  }
}

TEST(AeroTest, CenterOfPressureAboveCgPitchesNoseUpInForwardFlight) {
  const physics::AeroLoads loads = physics::aero_loads(
      params({0.01, 0.01, 0.01}, {0.0, 0.0, -0.02}), kRho, {20.0, 0.0, 0.0}, Vector3d::Zero());
  EXPECT_GT(loads.torque_frd.y(), 0.0);
  EXPECT_NEAR(loads.torque_frd.y(), 0.02 * 0.5 * kRho * 0.01 * 400.0, 1e-12);
}

TEST(AeroTest, AngularDampingOpposesBodyRates) {
  const physics::AeroLoads loads = physics::aero_loads(params({0.01, 0.01, 0.01}, Vector3d::Zero()),
                                                       kRho, Vector3d::Zero(), {1.0, -2.0, 3.0});
  EXPECT_NEAR(loads.torque_frd.x(), -0.001, 1e-15);
  EXPECT_NEAR(loads.torque_frd.y(), 0.004, 1e-15);
  EXPECT_NEAR(loads.torque_frd.z(), -0.009, 1e-15);
}
