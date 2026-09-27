#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include <fpvsim/video/pipeline.hpp>

namespace fpvsim::video {

struct VideoConfig {
  std::string host;
  std::uint16_t status_port;
  std::uint16_t control_port;
  std::vector<CameraSpec> cameras;
};

// Reads resolved/cameras.json; throws std::runtime_error naming the file and field on any problem
VideoConfig parse_cameras(const std::string& json_text, const std::filesystem::path& path);
VideoConfig load_cameras(const std::filesystem::path& path);

}  // namespace fpvsim::video
