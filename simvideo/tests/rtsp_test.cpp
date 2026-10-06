#include <gtest/gtest.h>

#include <fcntl.h>
#include <netinet/in.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <optional>
#include <sstream>
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
constexpr int kPort = 18554;  // off the 8554 default, so a running server does not clash

// FPVSIM_MEDIAMTX, or mediamtx on PATH
std::optional<std::filesystem::path> find_mediamtx() {
  if (const char* override = std::getenv("FPVSIM_MEDIAMTX"); override != nullptr) {
    return std::filesystem::path(override);
  }
  const char* path = std::getenv("PATH");
  if (path == nullptr) {
    return std::nullopt;
  }
  std::stringstream dirs(path);
  std::string dir;
  while (std::getline(dirs, dir, ':')) {
    const std::filesystem::path candidate = std::filesystem::path(dir) / "mediamtx";
    if (::access(candidate.c_str(), X_OK) == 0) {
      return candidate;
    }
  }
  return std::nullopt;
}

bool port_open(int port) {
  const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(static_cast<std::uint16_t>(port));
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  const bool open = ::connect(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) == 0;
  ::close(fd);
  return open;
}

// An RTSP-only server on kPort for the length of one test
class MediaMtx {
 public:
  explicit MediaMtx(const std::filesystem::path& binary) {
    dir_ =
        std::filesystem::temp_directory_path() / ("fpvsim_mediamtx_" + std::to_string(::getpid()));
    std::filesystem::create_directories(dir_);
    const std::filesystem::path config = dir_ / "mediamtx.yml";
    std::ofstream(config) << "logLevel: info\nrtspAddress: :" << kPort
                          << "\nrtspTransports: [tcp]\nrtmp: no\nhls: no\nwebrtc: no\nsrt: no\n"
                             "moq: no\npaths:\n  all_others:\n";
    pid_ = ::fork();
    if (pid_ == 0) {
      ::chdir(dir_.c_str());
      const int log = ::open("mediamtx.log", O_WRONLY | O_CREAT | O_TRUNC, 0600);
      ::dup2(log, STDOUT_FILENO);
      ::dup2(log, STDERR_FILENO);
      ::execl(binary.c_str(), "mediamtx", config.c_str(), static_cast<char*>(nullptr));
      ::_exit(127);
    }
  }
  ~MediaMtx() {
    stop();
    std::filesystem::remove_all(dir_);
  }
  void stop() {
    if (pid_ > 0) {
      ::kill(pid_, SIGTERM);
      ::waitpid(pid_, nullptr, 0);
      pid_ = -1;
    }
  }
  // what the server logged, to explain a failure
  [[nodiscard]] std::string log() const {
    std::ifstream file(dir_ / "mediamtx.log");
    return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
  }
  MediaMtx(const MediaMtx&) = delete;
  MediaMtx& operator=(const MediaMtx&) = delete;

  [[nodiscard]] bool wait_ready() const {
    for (int i = 0; i < 50; ++i) {
      if (port_open(kPort)) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return false;
  }

 private:
  std::filesystem::path dir_;
  pid_t pid_ = -1;
};

}  // namespace

TEST(RtspTest, RtspPushToMediaMtxIsReadBackAndDecoded) {
  video::init_gstreamer();
  for (const char* element : {"openh264enc", "openh264dec", "rtspclientsink", "rtspsrc"}) {
    GstElementFactory* factory = gst_element_factory_find(element);
    if (factory == nullptr) {
      GTEST_SKIP() << element << " is not installed";
    }
    gst_object_unref(factory);
  }
  const auto binary = find_mediamtx();
  if (!binary) {
    GTEST_SKIP() << "mediamtx is not on PATH (or set FPVSIM_MEDIAMTX)";
  }
  if (port_open(kPort)) {
    GTEST_SKIP() << "port " << kPort << " is already taken";
  }
  MediaMtx server(*binary);
  ASSERT_TRUE(server.wait_ready()) << "mediamtx did not start listening";

  const std::string url = "rtsp://127.0.0.1:" + std::to_string(kPort) + "/fpvsim";
  video::CameraRunner runner(video::CameraSpec{
      .name = "test_cam",
      .width = kWidth,
      .height = kHeight,
      .fps = 60.0,
      .pixel_format = proto::PixelFormat::kRgb8,
      .sensor_latency_s = 0.0,
      .outputs = {{.pipeline = "openh264enc ! h264parse ! rtspclientsink location=" + url +
                               " protocols=tcp",
                   .enabled = true}}});
  runner.start();
  ASSERT_EQ(runner.status().outputs.at(0).state, "running");

  // the reader can only attach once the push is announced, so frames keep flowing meanwhile
  std::atomic<bool> stop{false};
  std::thread feeder([&] {
    const std::vector<std::byte> pixels(static_cast<std::size_t>(kWidth) * kHeight * 3,
                                        std::byte{0x50});
    std::int64_t sim_ns = 0;
    while (!stop.load()) {
      runner.push_frame(pixels, sim_ns);
      runner.poll();
      sim_ns += 16'666'667;
      std::this_thread::sleep_for(std::chrono::nanoseconds(16'666'667));
    }
  });
  // the push is announced only once the encoder has produced its first frame, and a reader that
  // comes earlier is turned away, so retry the reader until the stream is up
  const std::string receiver_launch =
      "rtspsrc location=" + url +
      " protocols=tcp latency=50 ! rtph264depay ! h264parse ! openh264dec ! videoconvert ! "
      "video/x-raw,format=RGB ! appsink name=out sync=false max-buffers=8 drop=true";
  int decoded = 0;
  for (int attempt = 0; attempt < 10 && decoded < 10; ++attempt) {
    GError* error = nullptr;
    GstElement* receiver = gst_parse_launch(receiver_launch.c_str(), &error);
    ASSERT_NE(receiver, nullptr) << (error != nullptr ? error->message : "no pipeline");
    GstElement* sink = gst_bin_get_by_name(GST_BIN(receiver), "out");
    ASSERT_NE(sink, nullptr);
    ASSERT_NE(gst_element_set_state(receiver, GST_STATE_PLAYING), GST_STATE_CHANGE_FAILURE);
    for (int i = 0; i < 20 && decoded < 10; ++i) {
      GstSample* sample = gst_app_sink_try_pull_sample(GST_APP_SINK(sink), 100 * GST_MSECOND);
      if (sample != nullptr) {
        EXPECT_EQ(gst_buffer_get_size(gst_sample_get_buffer(sample)),
                  static_cast<gsize>(kWidth) * kHeight * 3);
        ++decoded;
        gst_sample_unref(sample);
      }
    }
    gst_element_set_state(receiver, GST_STATE_NULL);
    gst_object_unref(sink);
    gst_object_unref(receiver);
  }
  stop = true;
  feeder.join();
  EXPECT_EQ(runner.status().outputs.at(0).state, "running");
  runner.stop();
  server.stop();
  EXPECT_GE(decoded, 10) << "the RTSP stream never came back out of mediamtx:\n" << server.log();
}
