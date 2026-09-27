#include <gtest/gtest.h>

#include <string>

#include <nlohmann/json.hpp>

#include <fpvsim/video/status.hpp>

namespace video = fpvsim::video;

TEST(StatusTest, DatagramMatchesTheDocumentedShape) {
  const video::CameraStatus status{.camera = "main_fpv",
                                   .input_fps = 59.9,
                                   .late_frames = 2,
                                   .outputs = {{.index = 0,
                                                .pipeline = "v4l2sink device=/dev/video10",
                                                .state = "running",
                                                .fps = 59.9,
                                                .bitrate_bps = std::nullopt,
                                                .last_error = std::nullopt},
                                               {.index = 1,
                                                .pipeline = "x264enc ! udpsink port=5600",
                                                .state = "error",
                                                .fps = 0.0,
                                                .bitrate_bps = 4000.0,
                                                .last_error = "could not link"}}};
  const nlohmann::json json = nlohmann::json::parse(video::status_json(status));
  EXPECT_EQ(json["version"], 1);
  EXPECT_EQ(json["camera"], "main_fpv");
  EXPECT_DOUBLE_EQ(json["input_fps"].get<double>(), 59.9);
  EXPECT_EQ(json["late_frames"], 2);
  ASSERT_EQ(json["outputs"].size(), 2U);
  EXPECT_EQ(json["outputs"][0]["state"], "running");
  EXPECT_TRUE(json["outputs"][0]["bitrate_bps"].is_null());
  EXPECT_TRUE(json["outputs"][0]["last_error"].is_null());
  EXPECT_EQ(json["outputs"][1]["index"], 1);
  EXPECT_EQ(json["outputs"][1]["last_error"], "could not link");
  EXPECT_DOUBLE_EQ(json["outputs"][1]["bitrate_bps"].get<double>(), 4000.0);
}

TEST(StatusTest, LongPipelinesAreShortenedForDisplay) {
  video::CameraStatus status{.camera = "c", .input_fps = 0.0, .late_frames = 0, .outputs = {}};
  status.outputs.push_back({.index = 0,
                            .pipeline = std::string(400, 'x'),
                            .state = "running",
                            .fps = 0.0,
                            .bitrate_bps = std::nullopt,
                            .last_error = std::nullopt});
  const nlohmann::json json = nlohmann::json::parse(video::status_json(status));
  const auto shown = json["outputs"][0]["pipeline"].get<std::string>();
  EXPECT_EQ(shown.size(), video::kStatusPipelineChars);
  EXPECT_TRUE(shown.ends_with("..."));
}
