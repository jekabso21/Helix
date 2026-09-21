#include <gtest/gtest.h>

#include <algorithm>

#include <fpvsim/constants.hpp>
#include <fpvsim/physics/contact.hpp>

#include "test_quad.hpp"

namespace physics = fpvsim::physics;
namespace quad = fpvsim::testing;
using Eigen::Vector3d;

namespace {
constexpr double kGroundDownM = 0.0;
constexpr double kLegDropM = 0.02;

physics::RigidBodyState step(const physics::RigidBodyState& state, double dt) {
  const physics::ContactLoads contact =
      physics::contact_loads(quad::quad_contact(), state, kGroundDownM);
  physics::Loads loads = quad::no_loads();
  loads.force_ned = contact.force_ned;
  loads.torque_frd = contact.torque_frd;
  return physics::integrate(state, physics::derivatives(state, quad::quad_mass(), loads), dt);
}
}  // namespace

TEST(ContactTest, NoForceAboveGround) {
  const physics::ContactLoads loads = physics::contact_loads(
      quad::quad_contact(), physics::level_state_at({0.0, 0.0, -1.0}), kGroundDownM);
  EXPECT_FALSE(loads.touching);
  EXPECT_EQ(loads.force_ned.norm(), 0.0);
}

TEST(ContactTest, RestingPenetrationIsWeightOverTotalStiffness) {
  physics::RigidBodyState state = physics::level_state_at({0.0, 0.0, -kLegDropM});
  for (int i = 0; i < 5000; ++i) {
    state = step(state, 0.001);
  }
  const double expected = quad::kMassKg * fpvsim::kStandardGravityMps2 / (4.0 * 3000.0);
  EXPECT_NEAR(state.position_ned.z() + kLegDropM, expected, 1e-6);
  EXPECT_LT(state.velocity_ned.norm(), 1e-6);
  EXPECT_LT(state.angular_rate_frd.norm(), 1e-9);
}

TEST(ContactTest, BounceNeverReturnsHigherThanTheDropHeight) {
  const double drop_height = 0.5;
  physics::RigidBodyState state = physics::level_state_at({0.0, 0.0, -drop_height});
  bool touched = false;
  double highest_after_touch = 0.0;
  for (int i = 0; i < 4000; ++i) {
    state = step(state, 0.001);
    const bool touching =
        physics::contact_loads(quad::quad_contact(), state, kGroundDownM).touching;
    touched = touched || touching;
    if (touched && !touching) {
      highest_after_touch = std::max(highest_after_touch, -state.position_ned.z());
    }
  }
  ASSERT_TRUE(touched);
  EXPECT_LT(highest_after_touch, drop_height);
}

TEST(ContactTest, FrictionOpposesSlidingAndReportsClosingSpeed) {
  physics::RigidBodyState state = physics::level_state_at({0.0, 0.0, -kLegDropM + 0.001});
  state.velocity_ned = Vector3d(2.0, 0.0, 0.5);
  const physics::ContactLoads loads =
      physics::contact_loads(quad::quad_contact(), state, kGroundDownM);
  ASSERT_TRUE(loads.touching);
  EXPECT_LT(loads.force_ned.x(), 0.0);
  EXPECT_LT(loads.force_ned.z(), 0.0);
  EXPECT_NEAR(loads.max_closing_speed_mps, 0.5, 1e-12);
  EXPECT_NEAR(-loads.force_ned.x() / -loads.force_ned.z(), 0.6 * 2.0 / 2.01, 1e-9);
}
