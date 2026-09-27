#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <string>
#include <thread>
#include <vector>

#include <gst/app/gstappsink.h>
#include <gst/gst.h>

#include <fpvsim/video/runner.hpp>

namespace video = fpvsim::video;
namespace proto = fpvsim::proto;

namespace {

constexpr std::uint32_t kWidth = 32;
constexpr std::uint32_t kHeight = 24;

video::CameraSpec camera(std::vector<video::OutputSpec> outputs) {
  return video::CameraSpec{.name = "test_cam",
                           .width = kWidth,
                           .height = kHeight,
                           .fps = 60.0,
                           .pixel_format = proto::PixelFormat::kRgb8,
                           .sensor_latency_s = 0.0,
                           .outputs = std::move(outputs)};
}

std::filesystem::path temp_file(const std::string& suffix) {
  return std::filesystem::temp_directory_path() /
         ("fpvsim_video_" + std::to_string(::getpid()) + "_" + suffix + ".raw");
}

// Pushes frames and lets the pipelines drain them
void feed(video::CameraRunner& runner, int frames) {
  const std::vector<std::byte> pixels(static_cast<std::size_t>(kWidth) * kHeight * 3,
                                      std::byte{0x40});
  for (int i = 0; i < frames; ++i) {
    runner.push_frame(pixels, static_cast<std::int64_t>(i) * 16'666'667);
    runner.poll();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  runner.poll();
}

}  // namespace

TEST(RunnerTest, FramesReachAWorkingOutput) {
  const std::filesystem::path out = temp_file("good");
  std::filesystem::remove(out);
  {
    video::CameraRunner runner(
        camera({{.pipeline = "filesink location=" + out.string(), .enabled = true}}));
    runner.start();
    ASSERT_EQ(runner.status().outputs.at(0).state, "running");
    feed(runner, 5);
    runner.stop();
  }
  ASSERT_TRUE(std::filesystem::exists(out));
  EXPECT_EQ(std::filesystem::file_size(out), 5U * kWidth * kHeight * 3U);
  std::filesystem::remove(out);
}

// Acceptance: a broken output shows as an error while the other outputs keep running
TEST(RunnerTest, ABrokenOutputDoesNotStopTheOthers) {
  const std::filesystem::path out = temp_file("mixed");
  std::filesystem::remove(out);
  {
    video::CameraRunner runner(camera({
        {.pipeline = "filesink location=" + out.string(), .enabled = true},
        {.pipeline = "nosuchelement12345 name=broken", .enabled = true},
    }));
    runner.start();
    const video::CameraStatus started = runner.status();
    EXPECT_EQ(started.outputs.at(0).state, "running");
    EXPECT_EQ(started.outputs.at(1).state, "error");
    ASSERT_TRUE(started.outputs.at(1).last_error.has_value());
    EXPECT_FALSE(started.outputs.at(1).last_error->empty());

    feed(runner, 4);
    const video::CameraStatus after = runner.status();
    EXPECT_EQ(after.outputs.at(0).state, "running") << "the good output stopped too";
    EXPECT_EQ(after.outputs.at(1).state, "error");
    runner.stop();
  }
  ASSERT_TRUE(std::filesystem::exists(out));
  EXPECT_EQ(std::filesystem::file_size(out), 4U * kWidth * kHeight * 3U);
  std::filesystem::remove(out);
}

TEST(RunnerTest, DisabledOutputsNeverStart) {
  const std::filesystem::path out = temp_file("disabled");
  std::filesystem::remove(out);
  {
    video::CameraRunner runner(
        camera({{.pipeline = "filesink location=" + out.string(), .enabled = false}}));
    runner.start();
    EXPECT_EQ(runner.status().outputs.at(0).state, "disabled");
    feed(runner, 3);
    runner.stop();
  }
  EXPECT_FALSE(std::filesystem::exists(out));
}

TEST(RunnerTest, InputFpsAndLateFramesAreReported) {
  video::CameraRunner runner(camera({{.pipeline = "fakesink sync=false", .enabled = true}}));
  runner.start();
  runner.note_late_frame();
  runner.note_late_frame();
  feed(runner, 3);
  const video::CameraStatus status = runner.status();
  EXPECT_EQ(status.camera, "test_cam");
  EXPECT_EQ(status.late_frames, 2U);
  EXPECT_GE(status.input_fps, 0.0);
  runner.stop();
}

// Acceptance: an output type is received by a standard GStreamer consumer. Encodes to H.264,
// sends it over RTP/UDP and decodes it back, all through elements a normal player would use.
TEST(RunnerTest, RtpH264OutputIsReceivedAndDecoded) {
  video::init_gstreamer();
  for (const char* element : {"openh264enc", "openh264dec", "rtph264pay", "rtph264depay"}) {
    GstElementFactory* factory = gst_element_factory_find(element);
    if (factory == nullptr) {
      GTEST_SKIP() << element << " is not installed";
    }
    gst_object_unref(factory);
  }
  constexpr int kPort = 55611;  // unlikely to clash with a running session
  const std::string receiver_launch =
      "udpsrc port=" + std::to_string(kPort) +
      " caps=application/x-rtp,media=video,encoding-name=H264,payload=96 ! rtpjitterbuffer "
      "latency=50 ! rtph264depay ! h264parse ! openh264dec ! videoconvert ! "
      "video/x-raw,format=RGB ! appsink name=out sync=false max-buffers=8 drop=false";
  GError* error = nullptr;
  GstElement* receiver = gst_parse_launch(receiver_launch.c_str(), &error);
  ASSERT_NE(receiver, nullptr) << (error != nullptr ? error->message : "no pipeline");
  GstElement* sink = gst_bin_get_by_name(GST_BIN(receiver), "out");
  ASSERT_NE(sink, nullptr);
  ASSERT_NE(gst_element_set_state(receiver, GST_STATE_PLAYING), GST_STATE_CHANGE_FAILURE);

  {
    video::CameraRunner runner(camera({{.pipeline = "openh264enc ! h264parse config-interval=1 ! "
                                                    "rtph264pay config-interval=1 pt=96 ! udpsink "
                                                    "host=127.0.0.1 port=" +
                                                    std::to_string(kPort) + " sync=false",
                                        .enabled = true}}));
    runner.start();
    ASSERT_EQ(runner.status().outputs.at(0).state, "running");
    feed(runner, 30);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    runner.stop();
  }

  int decoded = 0;
  for (int i = 0; i < 40 && decoded < 5; ++i) {
    GstSample* sample = gst_app_sink_try_pull_sample(GST_APP_SINK(sink), 100 * GST_MSECOND);
    if (sample != nullptr) {
      GstBuffer* buffer = gst_sample_get_buffer(sample);
      EXPECT_EQ(gst_buffer_get_size(buffer), static_cast<gsize>(kWidth) * kHeight * 3);
      ++decoded;
      gst_sample_unref(sample);
    }
  }
  gst_element_set_state(receiver, GST_STATE_NULL);
  gst_object_unref(sink);
  gst_object_unref(receiver);
  EXPECT_GE(decoded, 5) << "the RTP stream never arrived at a standard receiver";
}

// Frames start arriving well after the pipeline went playing. A PTS counted from the first frame
// would then look seconds late to a clock-syncing sink (v4l2sink drops those and nothing reaches
// the device), so the first frame is placed at the current running time while the sim-time deltas
// between frames are kept exactly.
TEST(RunnerTest, FirstPtsIsPlacedAtRunningTimeAndDeltasAreKept) {
  video::CameraRunner runner(camera(
      {{.pipeline = "appsink name=out sync=false max-buffers=8 drop=false", .enabled = true}}));
  runner.start();
  std::this_thread::sleep_for(std::chrono::milliseconds(300));  // the pipeline clock runs on

  const std::vector<std::byte> pixels(static_cast<std::size_t>(kWidth) * kHeight * 3,
                                      std::byte{0x20});
  constexpr std::int64_t kPeriodNs = 16'666'667;

  GstElement* pipeline = nullptr;
  GstElement* sink = nullptr;
  ASSERT_TRUE(video::find_appsink_for_test(runner, 0, reinterpret_cast<void**>(&pipeline),
                                           reinterpret_cast<void**>(&sink)));
  // pull each frame before pushing the next: the branch queue is leaky by design
  std::vector<GstClockTime> stamps;
  for (int i = 0; i < 3; ++i) {
    runner.push_frame(pixels, static_cast<std::int64_t>(i) * kPeriodNs);
    GstSample* sample = gst_app_sink_try_pull_sample(GST_APP_SINK(sink), 500 * GST_MSECOND);
    ASSERT_NE(sample, nullptr) << "frame " << i << " never arrived";
    stamps.push_back(GST_BUFFER_PTS(gst_sample_get_buffer(sample)));
    gst_sample_unref(sample);
  }
  gst_object_unref(sink);
  runner.stop();

  EXPECT_GE(stamps.front(), 200 * GST_MSECOND) << "the first PTS was not moved to running time";
  EXPECT_EQ(stamps[1] - stamps[0], static_cast<GstClockTime>(kPeriodNs));
  EXPECT_EQ(stamps[2] - stamps[1], static_cast<GstClockTime>(kPeriodNs));
}
