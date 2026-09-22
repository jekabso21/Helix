#pragma once

#include <cstdint>

#include <nlohmann/json.hpp>

#include <fpvsim/config/session.hpp>

namespace fpvsim::sim {

struct RunSummary {
  std::int64_t steps;
  std::uint64_t overruns;
  std::uint64_t motor_packets;
  std::uint64_t malformed_packets;
  std::uint64_t dropped_log_rows;
  std::uint64_t render_states_sent;
  std::uint64_t api_requests;
  std::uint64_t esc_requests;
  std::uint64_t esc_answered;
  double final_height_m;
  bool crashed;
};

// Blocks until duration_s of sim time has elapsed; realtime paced with the wall clock
// Set by a signal handler; the loop exits cleanly at the next step
void request_stop() noexcept;

// info is what the control API returns for get_info
RunSummary run_realtime(const config::SessionConfig& session, const VehicleParams& drone,
                        const nlohmann::json& info);

}  // namespace fpvsim::sim
