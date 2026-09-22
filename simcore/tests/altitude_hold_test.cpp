#include <gtest/gtest.h>

#include <fpvsim/pilot/altitude_hold.hpp>

namespace pilot = fpvsim::pilot;

namespace {

pilot::AltitudeHoldParams params() {
  return pilot::AltitudeHoldParams{.target_height_m = 5.0,
                                   .climb_rate_mps = 1.0,
                                   .kp_us_per_m = 100.0,
                                   .ki_us_per_m_s = 20.0,
                                   .kd_us_per_mps = 80.0,
                                   .hover_throttle_us = 1350.0,
                                   .integral_limit_us = 300.0,
                                   .arm_delay_s = 1.0};
}

}  // namespace

TEST(AltitudeHoldTest, ArmSwitchGoesHighAfterTheDelayWithThrottleLow) {
  pilot::AltitudeHoldPilot p(params());
  const pilot::RcChannels before = p.channels(0.5, 0.0, 0.0, false, 0.001);
  const pilot::RcChannels after = p.channels(1.5, 0.0, 0.0, false, 0.001);
  EXPECT_EQ(before[4], 1000);
  EXPECT_EQ(after[4], 2000);
  EXPECT_EQ(after[2], 1000);
  EXPECT_EQ(after[0], 1500);
  EXPECT_EQ(after[3], 1500);
}

TEST(AltitudeHoldTest, ConvergesOnASimpleThrustPlant) {
  // 1D plant: acceleration proportional to throttle above a true hover point of 1400 us
  pilot::AltitudeHoldPilot p(params());
  const double dt = 0.001;
  double height = 0.0;
  double climb = 0.0;
  double t = 0.0;
  for (int i = 0; i < 30000; ++i) {
    const pilot::RcChannels channels = p.channels(t, height, climb, true, dt);
    const double acceleration = (channels[2] - 1400.0) * 0.02;
    climb += acceleration * dt;
    height += climb * dt;
    if (height < 0.0) {
      height = 0.0;
      climb = 0.0;
    }
    t += dt;
  }
  EXPECT_NEAR(height, 5.0, 0.1);
  EXPECT_NEAR(climb, 0.0, 0.05);
  EXPECT_DOUBLE_EQ(p.target_height_m(), 5.0);
}
