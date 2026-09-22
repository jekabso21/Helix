#include <fpvsim/input/mapping.hpp>

#include <algorithm>
#include <cmath>

namespace fpvsim::input {

namespace {
constexpr double kLowUs = 1000.0;
constexpr double kCentreUs = 1500.0;
constexpr double kHighUs = 2000.0;
constexpr std::size_t kStickChannels = 4;
}  // namespace

std::optional<std::size_t> channel_index(const std::string& name) {
  static constexpr std::array<const char*, kStickChannels> kSticks = {"roll", "pitch", "throttle",
                                                                      "yaw"};
  for (std::size_t i = 0; i < kSticks.size(); ++i) {
    if (name == kSticks[i]) {
      return i;
    }
  }
  if (name.size() >= 4 && name.starts_with("aux")) {
    const std::string digits = name.substr(3);
    if (digits.empty() ||
        !std::ranges::all_of(digits, [](char c) { return std::isdigit(c) != 0; })) {
      return std::nullopt;
    }
    const auto n = static_cast<std::size_t>(std::stoul(digits));
    if (n >= 1 && n <= kRcChannels - kStickChannels) {
      return kStickChannels + n - 1;
    }
  }
  return std::nullopt;
}

RcChannels map_channels(const InputMapping& mapping, const DeviceState& device) {
  RcChannels channels{};
  for (std::size_t i = 0; i < kRcChannels; ++i) {
    channels[i] = static_cast<std::uint16_t>(i < kStickChannels ? kCentreUs : kLowUs);
    const std::optional<ChannelSource>& source = mapping.channels[i];
    if (!source) {
      continue;
    }
    double us = kLowUs;
    if (source->kind == SourceKind::kAxis) {
      double value = source->index < kMaxAxes ? device.axes[source->index] : 0.0;
      if (std::abs(value) < source->deadband) {
        value = 0.0;
      }
      us = kCentreUs + (kHighUs - kLowUs) / 2.0 * value;
    } else if (source->index < kMaxButtons && device.buttons[source->index]) {
      us = kHighUs;
    }
    if (source->inverted) {
      us = kLowUs + kHighUs - us;
    }
    channels[i] = static_cast<std::uint16_t>(std::lround(std::clamp(us, kLowUs, kHighUs)));
  }
  return channels;
}

}  // namespace fpvsim::input
