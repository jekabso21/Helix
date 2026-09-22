#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include <fpvsim/proto/wire.hpp>

namespace fpvsim::proto {

inline constexpr std::uint32_t kMagic = 0x53565046;  // "FPVS"
inline constexpr std::uint16_t kProtocolVersion = 1;
inline constexpr std::uint16_t kMsgTypeRenderState = 1;
inline constexpr std::size_t kHeaderSize = 16;
inline constexpr std::size_t kRenderStatePayloadSize = 192;
inline constexpr std::size_t kRenderStateMessageSize = kHeaderSize + kRenderStatePayloadSize;
inline constexpr std::size_t kRenderStateMotors = 8;

struct RenderState {
  std::int64_t sim_time_ns;
  std::array<double, 3> position_ned;
  std::array<double, 4> q_ned_from_frd;
  std::array<double, 3> velocity_ned;
  std::array<double, 3> angular_rate_frd;
  std::uint8_t armed;
  std::uint8_t crashed;
  std::uint8_t motor_count;
  std::array<float, kRenderStateMotors> motor_rpm;
  std::array<float, 3> sun_dir_ned;
  float sun_intensity;
  float fog_density;
  std::uint8_t precip_type;
  float precip_intensity;
  std::array<float, 3> wind_ned;
  std::uint32_t video_fault_flags;
};

// Writes header plus payload; returns the number of bytes, 0 if the buffer is too small
inline std::size_t serialize_render_state(const RenderState& s, std::uint32_t seq,
                                          std::span<std::byte> out) {
  Writer w(out);
  w.put<std::uint32_t>(kMagic);
  w.put<std::uint16_t>(kProtocolVersion);
  w.put<std::uint16_t>(kMsgTypeRenderState);
  w.put<std::uint32_t>(kRenderStatePayloadSize);
  w.put<std::uint32_t>(seq);
  w.put<std::int64_t>(s.sim_time_ns);
  for (const double v : s.position_ned) {
    w.put_f64(v);
  }
  for (const double v : s.q_ned_from_frd) {
    w.put_f64(v);
  }
  for (const double v : s.velocity_ned) {
    w.put_f64(v);
  }
  for (const double v : s.angular_rate_frd) {
    w.put_f64(v);
  }
  w.put<std::uint8_t>(s.armed);
  w.put<std::uint8_t>(s.crashed);
  w.put<std::uint8_t>(s.motor_count);
  w.put<std::uint8_t>(0);
  for (const float v : s.motor_rpm) {
    w.put_f32(v);
  }
  for (const float v : s.sun_dir_ned) {
    w.put_f32(v);
  }
  w.put_f32(s.sun_intensity);
  w.put_f32(s.fog_density);
  w.put<std::uint8_t>(s.precip_type);
  w.put<std::uint8_t>(0);
  w.put<std::uint8_t>(0);
  w.put<std::uint8_t>(0);
  w.put_f32(s.precip_intensity);
  for (const float v : s.wind_ned) {
    w.put_f32(v);
  }
  w.put<std::uint32_t>(s.video_fault_flags);
  return w.overflowed() ? 0 : w.offset();
}

struct ParsedRenderState {
  std::uint32_t seq;
  RenderState state;
};

inline std::optional<ParsedRenderState> parse_render_state(std::span<const std::byte> in) {
  if (in.size() != kRenderStateMessageSize) {
    return std::nullopt;
  }
  Reader r(in);
  if (r.get<std::uint32_t>() != kMagic || r.get<std::uint16_t>() != kProtocolVersion ||
      r.get<std::uint16_t>() != kMsgTypeRenderState ||
      r.get<std::uint32_t>() != kRenderStatePayloadSize) {
    return std::nullopt;
  }
  ParsedRenderState p{};
  p.seq = r.get<std::uint32_t>();
  RenderState& s = p.state;
  s.sim_time_ns = r.get<std::int64_t>();
  for (double& v : s.position_ned) {
    v = r.get_f64();
  }
  for (double& v : s.q_ned_from_frd) {
    v = r.get_f64();
  }
  for (double& v : s.velocity_ned) {
    v = r.get_f64();
  }
  for (double& v : s.angular_rate_frd) {
    v = r.get_f64();
  }
  s.armed = r.get<std::uint8_t>();
  s.crashed = r.get<std::uint8_t>();
  s.motor_count = r.get<std::uint8_t>();
  r.get<std::uint8_t>();
  for (float& v : s.motor_rpm) {
    v = r.get_f32();
  }
  for (float& v : s.sun_dir_ned) {
    v = r.get_f32();
  }
  s.sun_intensity = r.get_f32();
  s.fog_density = r.get_f32();
  s.precip_type = r.get<std::uint8_t>();
  r.get<std::uint8_t>();
  r.get<std::uint8_t>();
  r.get<std::uint8_t>();
  s.precip_intensity = r.get_f32();
  for (float& v : s.wind_ned) {
    v = r.get_f32();
  }
  s.video_fault_flags = r.get<std::uint32_t>();
  if (r.failed()) {
    return std::nullopt;
  }
  return p;
}

}  // namespace fpvsim::proto
