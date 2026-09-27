#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <string>
#include <thread>
#include <vector>

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
