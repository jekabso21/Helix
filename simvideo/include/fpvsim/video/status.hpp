#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace fpvsim::video {

inline constexpr int kStatusVersion = 1;
inline constexpr std::size_t kStatusPipelineChars = 120;  // shortened for display

struct OutputStatus {
  int index;
  std::string pipeline;
  std::string state;  // starting, running, error, restarting, disabled
  double fps;
  std::optional<double> bitrate_bps;
  std::optional<std::string> last_error;
};

struct CameraStatus {
  std::string camera;
  double input_fps;
  std::uint64_t late_frames;
  std::vector<OutputStatus> outputs;
};

// One status datagram
std::string status_json(const CameraStatus& status);

}  // namespace fpvsim::video
