#include <fpvsim/video/config.hpp>

#include <fstream>
#include <sstream>
#include <stdexcept>

#include <nlohmann/json.hpp>

namespace fpvsim::video {

namespace {

constexpr int kSchemaVersion = 1;

[[noreturn]] void fail(const std::filesystem::path& path, const std::string& field,
                       const std::string& why) {
  throw std::runtime_error(path.filename().string() + ": " + field + ": " + why);
}

// Returns a pointer: a reference here would bind through a temporary at some call sites
const nlohmann::json* require(const nlohmann::json& node, const std::string& key,
                              const std::filesystem::path& path, const std::string& where) {
  if (!node.is_object() || !node.contains(key)) {
    fail(path, where + key, "missing");
  }
  return &node.at(key);
}

proto::PixelFormat pixel_format_from(const std::string& name, const std::filesystem::path& path) {
  if (name == "rgb8") {
    return proto::PixelFormat::kRgb8;
  }
  if (name == "rgba8") {
    return proto::PixelFormat::kRgba8;
  }
  fail(path, "pixel_format", "expected rgb8 or rgba8, got " + name);
}

}  // namespace

VideoConfig parse_cameras(const std::string& json_text, const std::filesystem::path& path) {
  const nlohmann::json document = nlohmann::json::parse(json_text, nullptr, false);
  if (document.is_discarded()) {
    fail(path, "(root)", "is not valid JSON");
  }
  if (require(document, "schema_version", path, "")->get<int>() != kSchemaVersion) {
    fail(path, "schema_version", "unsupported");
  }
  VideoConfig config;
  config.host = require(document, "host", path, "")->get<std::string>();
  config.status_port = require(document, "status_port", path, "")->get<std::uint16_t>();
  const nlohmann::json& cameras = *require(document, "cameras", path, "");
  if (!cameras.is_array() || cameras.empty()) {
    fail(path, "cameras", "expected a non-empty array");
  }
  for (const nlohmann::json& camera : cameras) {
    const std::string where = "cameras[]: ";
    CameraSpec spec;
    spec.name = require(camera, "name", path, where)->get<std::string>();
    spec.width = require(camera, "width", path, where)->get<std::uint32_t>();
    spec.height = require(camera, "height", path, where)->get<std::uint32_t>();
    spec.fps = require(camera, "fps", path, where)->get<double>();
    spec.pixel_format =
        pixel_format_from(require(camera, "pixel_format", path, where)->get<std::string>(), path);
    spec.sensor_latency_s = require(camera, "sensor_latency_s", path, where)->get<double>();
    if (spec.width == 0 || spec.height == 0 || spec.fps <= 0.0) {
      fail(path, where + spec.name, "needs a positive width, height and fps");
    }
    for (const nlohmann::json& output : *require(camera, "outputs", path, where)) {
      spec.outputs.push_back(OutputSpec{.pipeline = output.at("pipeline").get<std::string>(),
                                        .enabled = output.value("enabled", true)});
    }
    config.cameras.push_back(std::move(spec));
  }
  return config;
}

VideoConfig load_cameras(const std::filesystem::path& path) {
  std::ifstream file(path);
  if (!file) {
    throw std::runtime_error("cannot open " + path.string());
  }
  std::stringstream buffer;
  buffer << file.rdbuf();
  return parse_cameras(buffer.str(), path);
}

}  // namespace fpvsim::video
