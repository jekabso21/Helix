#pragma once

#include <cstdint>

#include <fpvsim/config/session.hpp>

namespace fpvsim::sim {

struct RunSummary {
  std::int64_t steps;
  std::uint64_t overruns;
  std::uint64_t motor_packets;
  std::uint64_t malformed_packets;
  std::uint64_t dropped_log_rows;
  double final_height_m;
  bool crashed;
};

// Blocks until duration_s of sim time has elapsed; realtime paced with the wall clock
RunSummary run_realtime(const config::SessionConfig& session, const VehicleParams& drone);

}  // namespace fpvsim::sim
