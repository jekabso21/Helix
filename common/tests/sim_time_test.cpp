#include <gtest/gtest.h>

#include <cstdint>

#include <fpvsim/sim_time.hpp>

using fpvsim::SimTime;

TEST(SimTimeTest, StepFor1000HzIsOneMillisecond) {
  EXPECT_EQ(fpvsim::step_from_rate_hz(1000), SimTime{1'000'000});
}

TEST(SimTimeTest, RateThatDoesNotDivideOneSecondIsRejected) {
  EXPECT_FALSE(fpvsim::step_from_rate_hz(3).has_value());
  EXPECT_FALSE(fpvsim::step_from_rate_hz(0).has_value());
  EXPECT_FALSE(fpvsim::step_from_rate_hz(-1000).has_value());
}

TEST(SimTimeTest, OneHourOfStepsAccumulatesWithoutDrift) {
  const SimTime step = fpvsim::step_from_rate_hz(8000).value_or(SimTime{});
  ASSERT_EQ(step.ns, 125'000);
  SimTime t{};
  const std::int64_t steps = 8000LL * 3600;
  for (std::int64_t i = 0; i < steps; ++i) {
    t += step;
  }
  EXPECT_EQ(t.ns, 3600 * fpvsim::kNanosecondsPerSecond);
  EXPECT_EQ(t, step * steps);
}

TEST(SimTimeTest, ToSecondsConvertsNanoseconds) {
  EXPECT_DOUBLE_EQ(fpvsim::to_seconds(SimTime{1'500'000'000}), 1.5);
}

TEST(SimTimeTest, SimTimesCompareByValue) {
  EXPECT_LT(SimTime{1}, SimTime{2});
  EXPECT_EQ(SimTime{5} - SimTime{2}, SimTime{3});
}
