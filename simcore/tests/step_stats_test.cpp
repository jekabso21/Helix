#include <gtest/gtest.h>

#include <fpvsim/sim/step_stats.hpp>

TEST(StepStatsTest, MeanMaxAndP99FromKnownSamples) {
  fpvsim::sim::StepStats stats;
  for (int i = 0; i < 99; ++i) {
    stats.add(100.0);
  }
  stats.add(2000.0);
  EXPECT_DOUBLE_EQ(stats.mean_us(), (99.0 * 100.0 + 2000.0) / 100.0);
  EXPECT_DOUBLE_EQ(stats.max_us(), 2000.0);
  EXPECT_LE(stats.p99_us(), 120.0);
  stats.add(2000.0);
  stats.add(2000.0);
  EXPECT_GE(stats.p99_us(), 2000.0);
}

TEST(StepStatsTest, EmptyAndOverflowAreSafe) {
  fpvsim::sim::StepStats stats;
  EXPECT_DOUBLE_EQ(stats.p99_us(), 0.0);
  EXPECT_DOUBLE_EQ(stats.mean_us(), 0.0);
  stats.add(1e9);
  stats.add(-5.0);
  EXPECT_DOUBLE_EQ(stats.max_us(), 1e9);
}
