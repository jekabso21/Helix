#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <fpvsim/proto/frame_ring.hpp>

namespace fpvsim::video {

struct OutputSpec {
  std::string pipeline;  // GStreamer fragment from the camera config, used verbatim
  bool enabled;
};

struct CameraSpec {
  std::string name;
  std::uint32_t width;
  std::uint32_t height;
  double fps;
  proto::PixelFormat pixel_format;
  double sensor_latency_s;
  std::vector<OutputSpec> outputs;
};

// "RGB" or "RGBA", as GStreamer names the raw formats
std::string gst_format_name(proto::PixelFormat format);

// GStreamer wants an exact fraction; 59.94 becomes 59940/1000
std::string framerate_fraction(double fps);

std::string caps_string(const CameraSpec& camera);

// appsrc ! tee, then one leaky queue and videoconvert per enabled output (section 5 of the
// interface doc). With no enabled output the frames still flow, into a fakesink.
std::string build_pipeline(const CameraSpec& camera);

}  // namespace fpvsim::video
