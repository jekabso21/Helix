#include "frame_publisher.hpp"

#include <godot_cpp/core/class_db.hpp>

#include <exception>

namespace fpvsim {

void FramePublisher::_bind_methods() {
  using namespace godot;
  ClassDB::bind_method(D_METHOD("open", "camera_name", "width", "height", "fps"),
                       &FramePublisher::open);
  ClassDB::bind_method(D_METHOD("close"), &FramePublisher::close);
  ClassDB::bind_method(D_METHOD("is_open"), &FramePublisher::is_open);
  ClassDB::bind_method(D_METHOD("last_error"), &FramePublisher::last_error);
  ClassDB::bind_method(D_METHOD("frame_bytes"), &FramePublisher::frame_bytes);
  ClassDB::bind_method(
      D_METHOD("publish", "pixels", "sim_time_ns", "frame_index", "position_ned",
               "q_ned_from_camera"),
      &FramePublisher::publish);
}

bool FramePublisher::open(const godot::String &camera_name, int width, int height, double fps) {
  close();
  const proto::RingSpec spec{.width = static_cast<std::uint32_t>(width),
                             .height = static_cast<std::uint32_t>(height),
                             .slot_count = 4,
                             .pixel_format = proto::PixelFormat::kRgb8,
                             .fps_nominal = fps};
  try {
    writer_ = std::make_unique<proto::FrameRingWriter>(camera_name.utf8().get_data(), spec);
    last_error_ = godot::String();
    return true;
  } catch (const std::exception &error) {
    writer_.reset();
    last_error_ = godot::String(error.what());
    return false;
  }
}

void FramePublisher::close() { writer_.reset(); }

int64_t FramePublisher::frame_bytes() const {
  return writer_ ? static_cast<int64_t>(writer_->frame_bytes()) : 0;
}

int64_t FramePublisher::publish(const godot::PackedByteArray &pixels, int64_t sim_time_ns,
                                int64_t frame_index, const godot::Vector3 &position_ned,
                                const godot::Quaternion &q_ned_from_camera) {
  if (!writer_ || static_cast<std::size_t>(pixels.size()) != writer_->frame_bytes()) {
    return 0;
  }
  const proto::FrameMeta meta{
      .sim_time_ns = sim_time_ns,
      .frame_index = static_cast<std::uint64_t>(frame_index),
      .camera_position_ned = {position_ned.x, position_ned.y, position_ned.z},
      .q_ned_from_camera = {q_ned_from_camera.w, q_ned_from_camera.x, q_ned_from_camera.y,
                            q_ned_from_camera.z}};
  const std::span<const std::byte> bytes(reinterpret_cast<const std::byte *>(pixels.ptr()),
                                         static_cast<std::size_t>(pixels.size()));
  try {
    return static_cast<int64_t>(writer_->write(meta, bytes));
  } catch (const std::exception &error) {
    last_error_ = godot::String(error.what());
    return 0;
  }
}

}  // namespace fpvsim
