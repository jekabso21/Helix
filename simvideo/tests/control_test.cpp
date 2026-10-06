#include <gtest/gtest.h>

#include <fpvsim/video/control.hpp>

namespace video = fpvsim::video;

TEST(ControlTest, ParsesAnOutputCommand) {
  const auto command =
      video::parse_control(R"({"version": 1, "camera": "main_fpv", "index": 2, "enabled": false})");
  ASSERT_TRUE(command.has_value());
  EXPECT_EQ(command->camera, "main_fpv");
  EXPECT_EQ(command->index, 2U);
  EXPECT_FALSE(command->enabled);
}

// Anything on the port can send a datagram; none of them may take the process down
TEST(ControlTest, IgnoresMalformedDatagramsWithoutThrowing) {
  const char* const malformed[] = {
      "",
      "not json",
      "[1, 2]",
      R"({"version": 2, "camera": "main_fpv", "index": 0, "enabled": true})",
      R"({"camera": "main_fpv", "index": 0, "enabled": true})",
      R"({"version": 1, "camera": 5, "index": 0, "enabled": true})",
      R"({"version": 1, "camera": null, "index": 0, "enabled": true})",
      R"({"version": 1, "camera": "main_fpv", "index": -1, "enabled": true})",
      R"({"version": 1, "camera": "main_fpv", "index": "0", "enabled": true})",
      R"({"version": 1, "camera": "main_fpv", "index": 0, "enabled": 1})",
      R"({"version": 1, "camera": "main_fpv", "enabled": true})",
      R"({"version": "1", "camera": "main_fpv", "index": 0, "enabled": true})",
  };
  for (const char* datagram : malformed) {
    std::optional<video::OutputCommand> command;
    EXPECT_NO_THROW(command = video::parse_control(datagram)) << datagram;
    EXPECT_FALSE(command.has_value()) << datagram;
  }
}
