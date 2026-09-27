#include <gtest/gtest.h>

#include <stdexcept>
#include <string>

#include <fpvsim/video/config.hpp>

namespace video = fpvsim::video;
namespace proto = fpvsim::proto;

namespace {

const char* kCameras = R"({
  "schema_version": 1,
  "host": "127.0.0.1",
  "status_port": 7730,
  "cameras": [
    {
      "name": "main_fpv", "width": 1280, "height": 720, "fps": 60.0,
      "pixel_format": "rgb8", "sensor_latency_s": 0.012,
      "outputs": [
        {"pipeline": "v4l2sink device=/dev/video10", "enabled": true},
        {"pipeline": "fakesink", "enabled": false}
      ]
    }
  ]
})";

}  // namespace

TEST(VideoConfigTest, ParsesACompleteCameraList) {
  const video::VideoConfig config = video::parse_cameras(kCameras, "cameras.json");
  EXPECT_EQ(config.host, "127.0.0.1");
  EXPECT_EQ(config.status_port, 7730);
  ASSERT_EQ(config.cameras.size(), 1U);
  const video::CameraSpec& camera = config.cameras.front();
  EXPECT_EQ(camera.name, "main_fpv");
  EXPECT_EQ(camera.width, 1280U);
  EXPECT_EQ(camera.pixel_format, proto::PixelFormat::kRgb8);
  EXPECT_DOUBLE_EQ(camera.sensor_latency_s, 0.012);
  ASSERT_EQ(camera.outputs.size(), 2U);
  EXPECT_TRUE(camera.outputs[0].enabled);
  EXPECT_FALSE(camera.outputs[1].enabled);
}

TEST(VideoConfigTest, MissingAndInvalidFieldsNameThemselves) {
  EXPECT_THROW(video::parse_cameras("{not json", "cameras.json"), std::runtime_error);
  EXPECT_THROW(video::parse_cameras(R"({"schema_version": 2})", "cameras.json"),
               std::runtime_error);
  std::string without_port = kCameras;
  without_port.replace(without_port.find("\"status_port\""), 13, "\"unused_port\"");
  EXPECT_THROW(video::parse_cameras(without_port, "cameras.json"), std::runtime_error);
  std::string bad_format = kCameras;
  bad_format.replace(bad_format.find("rgb8"), 4, "yuv2");
  EXPECT_THROW(video::parse_cameras(bad_format, "cameras.json"), std::runtime_error);
  std::string zero_width = kCameras;
  zero_width.replace(zero_width.find("\"width\": 1280"), 13, "\"width\": 0   ");
  EXPECT_THROW(video::parse_cameras(zero_width, "cameras.json"), std::runtime_error);
}
