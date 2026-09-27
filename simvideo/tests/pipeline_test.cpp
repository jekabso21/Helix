#include <gtest/gtest.h>

#include <string>

#include <fpvsim/video/pipeline.hpp>

namespace video = fpvsim::video;
namespace proto = fpvsim::proto;

namespace {

video::CameraSpec camera(std::vector<video::OutputSpec> outputs = {}) {
  return video::CameraSpec{.name = "main_fpv",
                           .width = 1280,
                           .height = 720,
                           .fps = 60.0,
                           .pixel_format = proto::PixelFormat::kRgb8,
                           .sensor_latency_s = 0.012,
                           .outputs = std::move(outputs)};
}

}  // namespace

TEST(PipelineTest, FormatNamesMatchGStreamer) {
  EXPECT_EQ(video::gst_format_name(proto::PixelFormat::kRgb8), "RGB");
  EXPECT_EQ(video::gst_format_name(proto::PixelFormat::kRgba8), "RGBA");
}

TEST(PipelineTest, FramerateIsAnExactFraction) {
  EXPECT_EQ(video::framerate_fraction(60.0), "60/1");
  EXPECT_EQ(video::framerate_fraction(30.0), "30/1");
  EXPECT_EQ(video::framerate_fraction(59.94), "59940/1000");
}

TEST(PipelineTest, CapsDescribeTheRing) {
  EXPECT_EQ(video::caps_string(camera()),
            "video/x-raw,format=RGB,width=1280,height=720,framerate=60/1");
}

TEST(PipelineTest, EachEnabledOutputBecomesItsOwnLeakyBranch) {
  const std::string built = video::build_pipeline(camera({
      {.pipeline = "v4l2sink device=/dev/video10", .enabled = true},
      {.pipeline = "x264enc ! udpsink port=5600", .enabled = true},
  }));
  EXPECT_NE(built.find("appsrc name=src is-live=true format=time do-timestamp=false"),
            std::string::npos);
  EXPECT_NE(built.find("! tee name=t"), std::string::npos);
  EXPECT_NE(built.find("t. ! queue name=q0 leaky=downstream max-size-buffers=2 ! videoconvert ! "
                       "v4l2sink device=/dev/video10"),
            std::string::npos);
  EXPECT_NE(built.find("t. ! queue name=q1 leaky=downstream max-size-buffers=2 ! videoconvert ! "
                       "x264enc ! udpsink port=5600"),
            std::string::npos);
  EXPECT_EQ(built.find("fakesink"), std::string::npos);
}

TEST(PipelineTest, DisabledOutputsAreLeftOutAndTheBranchesRenumber) {
  const std::string built = video::build_pipeline(camera({
      {.pipeline = "first", .enabled = false},
      {.pipeline = "second", .enabled = true},
  }));
  EXPECT_EQ(built.find("first"), std::string::npos);
  EXPECT_NE(built.find("queue name=q0 leaky=downstream max-size-buffers=2 ! videoconvert ! second"),
            std::string::npos);
}

TEST(PipelineTest, WithNoOutputsTheFramesStillFlowIntoAFakesink) {
  const std::string built = video::build_pipeline(camera());
  EXPECT_NE(built.find("fakesink sync=false"), std::string::npos);
  EXPECT_NE(built.find("tee name=t"), std::string::npos);
}
