#include <fpvsim/video/status.hpp>

#include <nlohmann/json.hpp>

namespace fpvsim::video {

namespace {

std::string shorten(const std::string& pipeline) {
  if (pipeline.size() <= kStatusPipelineChars) {
    return pipeline;
  }
  return pipeline.substr(0, kStatusPipelineChars - 3) + "...";
}

}  // namespace

std::string status_json(const CameraStatus& status) {
  nlohmann::json outputs = nlohmann::json::array();
  for (const OutputStatus& output : status.outputs) {
    outputs.push_back({{"index", output.index},
                       {"pipeline", shorten(output.pipeline)},
                       {"state", output.state},
                       {"fps", output.fps},
                       {"bitrate_bps", output.bitrate_bps ? nlohmann::json(*output.bitrate_bps)
                                                          : nlohmann::json(nullptr)},
                       {"last_error", output.last_error ? nlohmann::json(*output.last_error)
                                                        : nlohmann::json(nullptr)}});
  }
  const nlohmann::json document = {{"version", kStatusVersion},
                                   {"camera", status.camera},
                                   {"input_fps", status.input_fps},
                                   {"late_frames", status.late_frames},
                                   {"outputs", outputs}};
  return document.dump();
}

}  // namespace fpvsim::video
