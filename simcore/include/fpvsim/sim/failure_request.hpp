#pragma once

#include <cstddef>
#include <cstdint>

#include <nlohmann/json.hpp>

#include <fpvsim/sim/failures.hpp>

namespace fpvsim::sim {

// An inject_failure or clear_failure request, trivially copyable so it can ride in a Command
struct FailureCommand {
  FailureSpec spec;
  std::uint32_t id;    // assigned by the I/O thread when injecting; the one to clear otherwise
  double start_s;      // absolute sim time; negative: at the step that applies it
  double duration_s;   // negative: until cleared
  bool all;            // clear_failure: every failure
};

// {"type", "target": {"motor": 1..n} | {"sensor": "gyro"|"accel", "axis": "x"|"y"|"z"|"all"},
//  "params": {...}, "start_s"?, "duration_s"?}; throws std::invalid_argument naming the problem
FailureCommand failure_from_json(const nlohmann::json& params, std::size_t motor_count);
// {"failure_id": n} or {"all": true}
FailureCommand clear_from_json(const nlohmann::json& params);
// The listing shape: id, type, target, params, start_s, end_s (null: until cleared), in_effect
nlohmann::json failure_to_json(const ActiveFailure& failure, double time_s);

}  // namespace fpvsim::sim
