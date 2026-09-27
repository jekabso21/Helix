#pragma once

#include <cstdint>

namespace fpvsim::video {

// Sensor latency is modelled by holding each frame: a frame stamped sim_time is released once the
// realtime schedule has passed sim_time + latency. The buffer keeps the sim timeline as its PTS,
// so a consumer that looks at timestamps sees the moment the frame represents, not when it landed.
constexpr std::int64_t kNanosPerSecond = 1'000'000'000;

constexpr std::int64_t buffer_pts_ns(std::int64_t sim_time_ns, std::int64_t first_sim_time_ns) {
  const std::int64_t pts = sim_time_ns - first_sim_time_ns;
  return pts > 0 ? pts : 0;
}

constexpr std::int64_t latency_ns(double sensor_latency_s) {
  return static_cast<std::int64_t>(sensor_latency_s * static_cast<double>(kNanosPerSecond));
}

// Wall clock at which a frame may be pushed, anchored on the first frame seen
constexpr std::int64_t release_wall_ns(std::int64_t sim_time_ns, std::int64_t first_sim_time_ns,
                                       std::int64_t first_wall_ns, double sensor_latency_s) {
  return first_wall_ns + buffer_pts_ns(sim_time_ns, first_sim_time_ns) +
         latency_ns(sensor_latency_s);
}

// A frame is late when it could only be released after the next one was already due
constexpr bool is_late(std::int64_t release_ns, std::int64_t now_ns, double fps) {
  const auto period = fps > 0.0 ? static_cast<std::int64_t>(kNanosPerSecond / fps) : 0;
  return now_ns > release_ns + period;
}

}  // namespace fpvsim::video
