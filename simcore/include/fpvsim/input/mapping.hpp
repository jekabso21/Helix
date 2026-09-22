#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>

namespace fpvsim::input {

inline constexpr std::size_t kRcChannels = 16;
inline constexpr std::size_t kMaxAxes = 32;
inline constexpr std::size_t kMaxButtons = 64;

enum class SourceKind : std::uint8_t { kAxis, kButton };

struct ChannelSource {
  SourceKind kind;
  std::size_t index;
  bool inverted;
  double deadband;  // fraction of full travel, axes only
};

// Channel order is Betaflight's rc_packet order: A, E, T, R, AUX1..AUX12
struct InputMapping {
  std::string device_name_contains;
  std::array<std::optional<ChannelSource>, kRcChannels> channels;
  std::size_t arm_channel;
};

struct DeviceState {
  std::array<double, kMaxAxes> axes;  // -1..1
  std::array<bool, kMaxButtons> buttons;
};

using RcChannels = std::array<std::uint16_t, kRcChannels>;

// Sticks default to 1500 and aux channels to 1000 when unmapped
RcChannels map_channels(const InputMapping& mapping, const DeviceState& device);

// "roll", "pitch", "throttle", "yaw", "aux1".."aux12" to the channel index
std::optional<std::size_t> channel_index(const std::string& name);

}  // namespace fpvsim::input
