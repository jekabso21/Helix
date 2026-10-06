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

TEST(ScheduleTest, TheFirstAnchorStartsThePtsAtZero) {
  const std::int64_t sim = 4'000'000'000;
  const std::int64_t wall = 7'000'000'000;
  const video::Anchor anchor = video::anchor_at(sim, wall, 0);
  EXPECT_EQ(video::buffer_pts_ns(sim, anchor.first_sim_ns), 0);
  EXPECT_EQ(video::release_wall_ns(sim, anchor.first_sim_ns, anchor.first_wall_ns, 0.01),
            wall + 10'000'000);
}

// A restarted publisher counts its sim time from zero again. The outputs keep their clock, so the
// PTS must carry on after the last frame pushed rather than jump back, and the new frames must be
// released on time instead of being held until the old timeline catches up.
TEST(ScheduleTest, ReanchoringAfterARestartKeepsThePtsRising) {
  const video::Anchor before = video::anchor_at(9'000'000'000, 1'000'000'000, 0);
  const std::int64_t last_pts = video::buffer_pts_ns(29'000'000'000, before.first_sim_ns);
  ASSERT_EQ(last_pts, 20'000'000'000);

  const std::int64_t restarted_sim = 0;
  const std::int64_t now = 50'000'000'000;
  const std::int64_t next_pts = last_pts + video::frame_period_ns(60.0);
  const video::Anchor after = video::anchor_at(restarted_sim, now, next_pts);
  EXPECT_EQ(video::buffer_pts_ns(restarted_sim, after.first_sim_ns), next_pts);
  EXPECT_GT(video::buffer_pts_ns(restarted_sim + 16'666'667, after.first_sim_ns), next_pts);
  EXPECT_EQ(video::release_wall_ns(restarted_sim, after.first_sim_ns, after.first_wall_ns, 0.0),
            now);
}
