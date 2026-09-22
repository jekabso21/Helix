#include <fpvsim/input/mapping_json.hpp>

#include <array>
#include <stdexcept>
#include <string>

namespace fpvsim::input {

namespace {

void require(const nlohmann::json& node, const char* key, const std::string& path) {
  if (!node.is_object() || !node.contains(key)) {
    throw std::runtime_error(path + ": missing field '" + key + "'");
  }
}

const char* channel_name(std::size_t index) {
  static constexpr std::array<const char*, kRcChannels> kNames = {
      "roll", "pitch", "throttle", "yaw",  "aux1", "aux2",  "aux3",  "aux4",
      "aux5", "aux6",  "aux7",     "aux8", "aux9", "aux10", "aux11", "aux12"};
  return kNames[index];
}

}  // namespace

InputMapping mapping_from_json(const nlohmann::json& node) {
  const std::string path = "mapping";
  InputMapping mapping{};
  require(node, "device_name_contains", path);
  require(node, "channels", path);
  require(node, "arm_channel", path);
  mapping.device_name_contains = node["device_name_contains"].get<std::string>();
  const nlohmann::json& channels = node["channels"];
  if (!channels.is_object()) {
    throw std::runtime_error(path + ".channels: expected an object");
  }
  for (const auto& item : channels.items()) {
    const std::optional<std::size_t> index = channel_index(item.key());
    if (!index) {
      throw std::runtime_error(path + ".channels: unknown channel '" + item.key() + "'");
    }
    const nlohmann::json& source = item.value();
    const std::string source_path = path + ".channels." + item.key();
    const bool is_axis = source.is_object() && source.contains("axis");
    const bool is_button = source.is_object() && source.contains("button");
    if (is_axis == is_button) {
      throw std::runtime_error(source_path + ": exactly one of axis or button");
    }
    mapping.channels[*index] =
        ChannelSource{.kind = is_axis ? SourceKind::kAxis : SourceKind::kButton,
                      .index = source[is_axis ? "axis" : "button"].get<std::size_t>(),
                      .inverted = source.value("inverted", false),
                      .deadband = source.value("deadband", 0.0)};
  }
  const std::optional<std::size_t> arm = channel_index(node["arm_channel"].get<std::string>());
  if (!arm) {
    throw std::runtime_error(path + ".arm_channel: unknown channel");
  }
  mapping.arm_channel = *arm;
  return mapping;
}

nlohmann::json mapping_to_json(const InputMapping& mapping) {
  nlohmann::json channels = nlohmann::json::object();
  for (std::size_t i = 0; i < kRcChannels; ++i) {
    const std::optional<ChannelSource>& source = mapping.channels[i];
    if (!source) {
      continue;
    }
    nlohmann::json entry = {{"inverted", source->inverted}, {"deadband", source->deadband}};
    entry[source->kind == SourceKind::kAxis ? "axis" : "button"] = source->index;
    channels[channel_name(i)] = entry;
  }
  return {{"device_name_contains", mapping.device_name_contains},
          {"arm_channel", channel_name(mapping.arm_channel)},
          {"channels", channels}};
}

}  // namespace fpvsim::input
