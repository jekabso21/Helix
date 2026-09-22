#include <gtest/gtest.h>

#include <stdexcept>

#include <fpvsim/input/mapping_json.hpp>

namespace input = fpvsim::input;

TEST(MappingJsonTest, RoundTripsThroughJson) {
  const nlohmann::json doc = nlohmann::json::parse(R"({
    "device_name_contains": "Boxer", "arm_channel": "aux1",
    "channels": {"throttle": {"axis": 0, "inverted": false, "deadband": 0.0},
                 "roll": {"axis": 1, "inverted": true, "deadband": 0.05},
                 "aux1": {"button": 0, "inverted": false, "deadband": 0.0}}})");
  const input::InputMapping mapping = input::mapping_from_json(doc);
  EXPECT_EQ(mapping.arm_channel, 4U);
  ASSERT_TRUE(mapping.channels[0].has_value());
  const input::ChannelSource roll = mapping.channels[0].value_or(input::ChannelSource{});
  EXPECT_TRUE(roll.inverted);
  EXPECT_DOUBLE_EQ(roll.deadband, 0.05);
  EXPECT_EQ(input::mapping_to_json(mapping), doc);
}

TEST(MappingJsonTest, DefaultsInvertedAndDeadbandAndRejectsBadInput) {
  const nlohmann::json minimal = nlohmann::json::parse(
      R"({"device_name_contains": "", "arm_channel": "aux2", "channels": {"yaw": {"axis": 3}}})");
  const input::InputMapping mapping = input::mapping_from_json(minimal);
  const input::ChannelSource yaw = mapping.channels[3].value_or(input::ChannelSource{
      .kind = input::SourceKind::kAxis, .index = 0, .inverted = true, .deadband = 0.0});
  EXPECT_FALSE(yaw.inverted);
  EXPECT_EQ(mapping.arm_channel, 5U);
  EXPECT_THROW(
      input::mapping_from_json(nlohmann::json::parse(R"({"arm_channel": "aux1", "channels": {}})")),
      std::runtime_error);
  EXPECT_THROW(
      input::mapping_from_json(nlohmann::json::parse(
          R"({"device_name_contains": "", "arm_channel": "aux1", "channels": {"gear": {"axis": 1}}})")),
      std::runtime_error);
  EXPECT_THROW(
      input::mapping_from_json(nlohmann::json::parse(
          R"({"device_name_contains": "", "arm_channel": "aux1", "channels": {"roll": {"axis": 1, "button": 2}}})")),
      std::runtime_error);
}
