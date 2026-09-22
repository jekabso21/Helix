#include <gtest/gtest.h>

#include <fpvsim/input/mapping.hpp>

namespace input = fpvsim::input;

namespace {

input::InputMapping boxer() {
  input::InputMapping m{};
  m.device_name_contains = "Radiomaster Boxer";
  m.channels[0] = {
      .kind = input::SourceKind::kAxis, .index = 1, .inverted = false, .deadband = 0.0};
  m.channels[1] = {
      .kind = input::SourceKind::kAxis, .index = 2, .inverted = false, .deadband = 0.0};
  m.channels[2] = {
      .kind = input::SourceKind::kAxis, .index = 0, .inverted = false, .deadband = 0.0};
  m.channels[3] = {
      .kind = input::SourceKind::kAxis, .index = 3, .inverted = false, .deadband = 0.05};
  m.channels[4] = {
      .kind = input::SourceKind::kButton, .index = 0, .inverted = true, .deadband = 0.0};
  m.channels[5] = {.kind = input::SourceKind::kAxis, .index = 4, .inverted = true, .deadband = 0.0};
  m.arm_channel = 4;
  return m;
}

}  // namespace

TEST(MappingTest, ChannelNamesFollowBetaflightOrder) {
  EXPECT_EQ(input::channel_index("roll"), 0U);
  EXPECT_EQ(input::channel_index("pitch"), 1U);
  EXPECT_EQ(input::channel_index("throttle"), 2U);
  EXPECT_EQ(input::channel_index("yaw"), 3U);
  EXPECT_EQ(input::channel_index("aux1"), 4U);
  EXPECT_EQ(input::channel_index("aux12"), 15U);
  EXPECT_FALSE(input::channel_index("aux13").has_value());
  EXPECT_FALSE(input::channel_index("aux").has_value());
  EXPECT_FALSE(input::channel_index("auxx").has_value());
  EXPECT_FALSE(input::channel_index("gear").has_value());
}

TEST(MappingTest, AxesScaleLinearlyAndButtonsSnap) {
  input::DeviceState device{};
  device.axes[0] = -1.0;  // throttle down
  device.axes[1] = 0.5;   // roll half right
  device.axes[2] = 1.0;   // pitch full forward
  device.buttons[0] = true;
  const input::RcChannels rc = input::map_channels(boxer(), device);
  EXPECT_EQ(rc[2], 1000);
  EXPECT_EQ(rc[0], 1750);
  EXPECT_EQ(rc[1], 2000);
  EXPECT_EQ(rc[3], 1500);
  EXPECT_EQ(rc[4], 1000);  // button pressed, inverted
  device.buttons[0] = false;
  EXPECT_EQ(input::map_channels(boxer(), device)[4], 2000);
}

TEST(MappingTest, DeadbandAndInvertedAxis) {
  input::DeviceState device{};
  device.axes[3] = 0.03;
  device.axes[4] = 1.0;
  const input::RcChannels rc = input::map_channels(boxer(), device);
  EXPECT_EQ(rc[3], 1500);
  EXPECT_EQ(rc[5], 1000);
  device.axes[3] = 0.2;
  device.axes[4] = -0.5;
  const input::RcChannels moved = input::map_channels(boxer(), device);
  EXPECT_EQ(moved[3], 1600);
  EXPECT_EQ(moved[5], 1750);
}

TEST(MappingTest, UnmappedChannelsUseNeutralDefaults) {
  const input::RcChannels rc = input::map_channels(boxer(), input::DeviceState{});
  EXPECT_EQ(rc[6], 1000);
  EXPECT_EQ(rc[15], 1000);
  input::InputMapping empty{};
  const input::RcChannels defaults = input::map_channels(empty, input::DeviceState{});
  EXPECT_EQ(defaults[0], 1500);
  EXPECT_EQ(defaults[2], 1500);
  EXPECT_EQ(defaults[4], 1000);
}

TEST(MappingTest, OutOfRangeIndicesReadAsNeutral) {
  input::InputMapping m{};
  m.channels[0] = {
      .kind = input::SourceKind::kAxis, .index = 99, .inverted = false, .deadband = 0.0};
  m.channels[4] = {
      .kind = input::SourceKind::kButton, .index = 99, .inverted = false, .deadband = 0.0};
  const input::RcChannels rc = input::map_channels(m, input::DeviceState{});
  EXPECT_EQ(rc[0], 1500);
  EXPECT_EQ(rc[4], 1000);
}
