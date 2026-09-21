#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

#include <fpvsim/constants.hpp>
#include <fpvsim/physics/rigid_body.hpp>

#include "test_quad.hpp"

namespace physics = fpvsim::physics;
using Eigen::Vector3d;

TEST(RigidBodyTest, FreeFallVelocityIsGravityTimesTime) {
  const physics::MassProperties mass = fpvsim::testing::quad_mass();
  physics::RigidBodyState state = physics::level_state_at(Vector3d(0.0, 0.0, -100.0));
  const double dt = 0.001;
  const int steps = 1000;
  for (int i = 0; i < steps; ++i) {
    state = physics::integrate(state,
                               physics::derivatives(state, mass, fpvsim::testing::no_loads()), dt);
  }
  EXPECT_NEAR(state.velocity_ned.z(), fpvsim::kStandardGravityMps2, 1e-9);
  const double semi_implicit_drop =
      fpvsim::kStandardGravityMps2 * dt * dt * steps * (steps + 1) / 2;
  EXPECT_NEAR(state.position_ned.z(), -100.0 + semi_implicit_drop, 1e-9);
  EXPECT_NEAR(state.position_ned.z(), -100.0 + 0.5 * fpvsim::kStandardGravityMps2, 0.01);
}

TEST(RigidBodyTest, TorqueFreeSpinAboutPrincipalAxisKeepsRate) {
  const physics::MassProperties mass = fpvsim::testing::quad_mass();
  physics::RigidBodyState state = physics::level_state_at(Vector3d::Zero());
  state.angular_rate_frd = Vector3d(0.0, 0.0, 5.0);
  for (int i = 0; i < 2000; ++i) {
    state = physics::integrate(
        state, physics::derivatives(state, mass, fpvsim::testing::no_loads()), 0.001);
  }
  EXPECT_LT((state.angular_rate_frd - Vector3d(0.0, 0.0, 5.0)).norm(), 1e-12);
}

TEST(RigidBodyTest, AsymmetricTorqueFreeRotationConservesMomentumAndEnergy) {
  const Eigen::Matrix3d inertia = Vector3d(0.002, 0.003, 0.005).asDiagonal();
  const physics::MassProperties mass = physics::make_mass_properties(0.5, inertia);
  physics::RigidBodyState state = physics::level_state_at(Vector3d::Zero());
  state.angular_rate_frd = Vector3d(3.0, 0.5, 0.2);
  const double momentum_before = (inertia * state.angular_rate_frd).norm();
  const double energy_before = 0.5 * state.angular_rate_frd.dot(inertia * state.angular_rate_frd);
  for (int i = 0; i < 10000; ++i) {
    state = physics::integrate(
        state, physics::derivatives(state, mass, fpvsim::testing::no_loads()), 0.0001);
  }
  const double momentum_after = (inertia * state.angular_rate_frd).norm();
  const double energy_after = 0.5 * state.angular_rate_frd.dot(inertia * state.angular_rate_frd);
  EXPECT_NEAR(momentum_after / momentum_before, 1.0, 1e-3);
  EXPECT_NEAR(energy_after / energy_before, 1.0, 1e-3);
}

TEST(RigidBodyTest, QuaternionNormStaysOneWhileTumbling) {
  const physics::MassProperties mass = fpvsim::testing::quad_mass();
  physics::RigidBodyState state = physics::level_state_at(Vector3d::Zero());
  state.angular_rate_frd = Vector3d(12.0, -7.0, 9.0);
  physics::Loads loads = fpvsim::testing::no_loads();
  loads.torque_frd = Vector3d(0.01, -0.02, 0.005);
  for (int i = 0; i < 5000; ++i) {
    state = physics::integrate(state, physics::derivatives(state, mass, loads), 0.001);
    ASSERT_NEAR(state.q_ned_from_frd.norm(), 1.0, 1e-12);
  }
}

TEST(RigidBodyTest, PositiveYawRateTurnsNoseFromNorthToEast) {
  const physics::MassProperties mass = fpvsim::testing::quad_mass();
  physics::RigidBodyState state = physics::level_state_at(Vector3d::Zero());
  state.angular_rate_frd = Vector3d(0.0, 0.0, std::numbers::pi / 2.0);
  physics::Loads hover = fpvsim::testing::no_loads();
  hover.force_frd = Vector3d(0.0, 0.0, -mass.mass_kg * fpvsim::kStandardGravityMps2);
  for (int i = 0; i < 1000; ++i) {
    state = physics::integrate(state, physics::derivatives(state, mass, hover), 0.001);
  }
  const Vector3d nose_ned = state.q_ned_from_frd * Vector3d(1.0, 0.0, 0.0);
  EXPECT_LT((nose_ned - Vector3d(0.0, 1.0, 0.0)).norm(), 1e-9);
  EXPECT_LT(state.velocity_ned.norm(), 1e-9);
}

TEST(RigidBodyTest, BodyForceIsRotatedIntoTheWorldFrame) {
  const physics::MassProperties mass = fpvsim::testing::quad_mass();
  physics::RigidBodyState state = physics::level_state_at(Vector3d::Zero());
  state.q_ned_from_frd =
      Eigen::Quaterniond(Eigen::AngleAxisd(std::numbers::pi / 2.0, Vector3d(0.0, 0.0, 1.0)));
  physics::Loads loads = fpvsim::testing::no_loads();
  loads.force_frd = Vector3d(1.0, 0.0, 0.0);
  const physics::Derivatives rates = physics::derivatives(state, mass, loads);
  EXPECT_NEAR(rates.acceleration_ned.y(), 1.0 / mass.mass_kg, 1e-12);
  EXPECT_NEAR(rates.acceleration_ned.x(), 0.0, 1e-12);
}
