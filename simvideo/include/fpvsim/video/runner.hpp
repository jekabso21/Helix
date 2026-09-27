#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <vector>

#include <fpvsim/video/pipeline.hpp>
#include <fpvsim/video/status.hpp>

namespace fpvsim::video {

// One GStreamer pipeline per output. The interface doc's single tee pipeline cannot meet its own
// requirement that a failing branch not affect the others: a GStreamer error stops the pipeline it
// belongs to, so each output gets its own with its own appsrc.
class OutputBranch {
 public:
  OutputBranch(const CameraSpec& camera, OutputSpec output, int index);
  ~OutputBranch();
  OutputBranch(OutputBranch&&) noexcept;
  OutputBranch& operator=(OutputBranch&&) = delete;
  OutputBranch(const OutputBranch&) = delete;
  OutputBranch& operator=(const OutputBranch&) = delete;

  void start();
  void stop();
  // Takes one reference of the buffer; does nothing unless the branch is running
  void push(void* buffer);
  // Drains the bus and restarts a failed branch once its backoff has passed
  void poll(std::chrono::steady_clock::time_point now);
  [[nodiscard]] OutputStatus status(std::chrono::steady_clock::time_point now);
  [[nodiscard]] const std::string& description() const { return description_; }

 private:
  void fail(const std::string& message, std::chrono::steady_clock::time_point now);

  OutputSpec output_;
  int index_;
  std::string description_;
  std::string caps_;
  void* pipeline_ = nullptr;
  void* appsrc_ = nullptr;
  std::string state_ = "disabled";
  std::string last_error_;
  std::uint64_t pushed_ = 0;
  std::uint64_t pushed_at_window_ = 0;
  std::chrono::steady_clock::time_point window_start_{};
  std::chrono::steady_clock::time_point retry_at_{};
  std::chrono::milliseconds backoff_{500};
  double fps_ = 0.0;
};

// Owns the branches of one camera and turns frames into buffers exactly once
class CameraRunner {
 public:
  explicit CameraRunner(CameraSpec camera);
  ~CameraRunner();
  CameraRunner(const CameraRunner&) = delete;
  CameraRunner& operator=(const CameraRunner&) = delete;

  void start();
  void stop();
  void push_frame(std::span<const std::byte> pixels, std::int64_t pts_ns);
  void poll();
  void note_late_frame() { ++late_frames_; }
  [[nodiscard]] CameraStatus status();
  [[nodiscard]] const CameraSpec& camera() const { return camera_; }

 private:
  CameraSpec camera_;
  std::vector<std::unique_ptr<OutputBranch>> branches_;
  std::uint64_t frames_ = 0;
  std::uint64_t frames_at_window_ = 0;
  std::uint64_t late_frames_ = 0;
  std::chrono::steady_clock::time_point window_start_{};
  double input_fps_ = 0.0;
};

// Calls gst_init once; safe to call repeatedly
void init_gstreamer();

}  // namespace fpvsim::video
