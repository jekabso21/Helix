#include <fpvsim/video/control.hpp>

#include <nlohmann/json.hpp>

namespace fpvsim::video {

std::optional<OutputCommand> parse_control(std::string_view datagram) {
  const nlohmann::json message =
      nlohmann::json::parse(datagram.begin(), datagram.end(), nullptr, false);
  if (message.is_discarded() || !message.is_object()) {
    return std::nullopt;
  }
  const auto version = message.find("version");
  const auto camera = message.find("camera");
  const auto index = message.find("index");
  const auto enabled = message.find("enabled");
  if (version == message.end() || !version->is_number_unsigned() || *version != 1 ||
      camera == message.end() || !camera->is_string() || index == message.end() ||
      !index->is_number_unsigned() || enabled == message.end() || !enabled->is_boolean()) {
    return std::nullopt;
  }
  return OutputCommand{.camera = camera->get<std::string>(),
                       .index = index->get<std::size_t>(),
                       .enabled = enabled->get<bool>()};
}

}  // namespace fpvsim::video
