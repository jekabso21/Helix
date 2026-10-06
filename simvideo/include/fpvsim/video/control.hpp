#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <string_view>

namespace fpvsim::video {

struct OutputCommand {
  std::string camera;
  std::size_t index;
  bool enabled;
};

// {"version": 1, "camera": "main_fpv", "index": 0, "enabled": false}; nullopt for anything else
std::optional<OutputCommand> parse_control(std::string_view datagram);

}  // namespace fpvsim::video
