#pragma once

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/variant/packed_byte_array.hpp>
#include <godot_cpp/variant/quaternion.hpp>
#include <godot_cpp/variant/string.hpp>
#include <godot_cpp/variant/vector3.hpp>

#include <memory>

#include <fpvsim/proto/frame_ring.hpp>

namespace fpvsim {

// Writes rendered camera frames into the shared memory ring simvideo reads
class FramePublisher : public godot::RefCounted {
  GDCLASS(FramePublisher, godot::RefCounted)

 public:
  FramePublisher() = default;
  ~FramePublisher() override = default;

  // rgb8 pixels; returns false and leaves an error message when the ring cannot be created
  bool open(const godot::String &camera_name, int width, int height, double fps);
  void close();
  bool is_open() const { return writer_ != nullptr; }
  godot::String last_error() const { return last_error_; }

  // Returns the published sequence number, or 0 when the frame was the wrong size
  int64_t publish(const godot::PackedByteArray &pixels, int64_t sim_time_ns, int64_t frame_index,
                  const godot::Vector3 &position_ned, const godot::Quaternion &q_ned_from_camera);

  int64_t frame_bytes() const;

 protected:
  static void _bind_methods();

 private:
  std::unique_ptr<proto::FrameRingWriter> writer_;
  godot::String last_error_;
};

}  // namespace fpvsim
