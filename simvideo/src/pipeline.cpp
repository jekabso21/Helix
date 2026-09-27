#include <fpvsim/video/pipeline.hpp>

#include <cmath>

namespace fpvsim::video {

std::string gst_format_name(proto::PixelFormat format) {
  return format == proto::PixelFormat::kRgba8 ? "RGBA" : "RGB";
}

std::string framerate_fraction(double fps) {
  const double rounded = std::round(fps);
  if (std::abs(fps - rounded) < 1e-6 && rounded > 0.0) {
    return std::to_string(static_cast<long long>(rounded)) + "/1";
  }
  return std::to_string(static_cast<long long>(std::llround(fps * 1000.0))) + "/1000";
}

std::string caps_string(const CameraSpec& camera) {
  return "video/x-raw,format=" + gst_format_name(camera.pixel_format) +
         ",width=" + std::to_string(camera.width) + ",height=" + std::to_string(camera.height) +
         ",framerate=" + framerate_fraction(camera.fps);
}

std::string build_pipeline(const CameraSpec& camera) {
  std::string pipeline =
      "appsrc name=src is-live=true format=time do-timestamp=false caps=" + caps_string(camera) +
      " ! tee name=t";
  int branch = 0;
  for (const OutputSpec& output : camera.outputs) {
    if (!output.enabled) {
      continue;
    }
    pipeline += " t. ! queue name=q" + std::to_string(branch) +
                " leaky=downstream max-size-buffers=2 ! videoconvert ! " + output.pipeline;
    ++branch;
  }
  if (branch == 0) {
    pipeline += " t. ! queue leaky=downstream max-size-buffers=2 ! fakesink sync=false";
  }
  return pipeline;
}

}  // namespace fpvsim::video
