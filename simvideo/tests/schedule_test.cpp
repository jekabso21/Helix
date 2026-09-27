#include <gtest/gtest.h>

#include <fpvsim/video/schedule.hpp>

namespace video = fpvsim::video;

TEST(ScheduleTest, PtsFollowsSimTimeFromTheFirstFrame) {
  const std::int64_t first = 5'000'000'000;
  EXPECT_EQ(video::buffer_pts_ns(first, first), 0);
  EXPECT_EQ(video::buffer_pts_ns(first + 16'666'667, first), 16'666'667);
  EXPECT_EQ(video::buffer_pts_ns(first - 1000, first), 0);  // never negative
}

TEST(ScheduleTest, SensorLatencyHoldsTheFrameWithoutMovingItsTimestamp) {
  const std::int64_t first_sim = 1'000'000'000;
  const std::int64_t first_wall = 900'000'000;
  const double latency = 0.012;
  const std::int64_t sim = first_sim + 100'000'000;  // 100 ms of sim time later
  EXPECT_EQ(video::latency_ns(latency), 12'000'000);
  EXPECT_EQ(video::release_wall_ns(sim, first_sim, first_wall, latency),
            first_wall + 100'000'000 + 12'000'000);
  // the timestamp the consumer sees is the moment the frame represents, not the release time
  EXPECT_EQ(video::buffer_pts_ns(sim, first_sim), 100'000'000);
}

TEST(ScheduleTest, AFrameIsLateOnlyOncePastTheNextFramesSlot) {
  const std::int64_t release = 1'000'000'000;
  EXPECT_FALSE(video::is_late(release, release, 60.0));
  EXPECT_FALSE(video::is_late(release, release + 16'000'000, 60.0));
  EXPECT_TRUE(video::is_late(release, release + 20'000'000, 60.0));
}
