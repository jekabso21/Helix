#pragma once

#include <array>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <thread>

#include <fpvsim/spsc_queue.hpp>

namespace fpvsim::log {

struct TruthRow {
  std::int64_t sim_time_ns;
  std::array<double, 3> position_ned;
  std::array<double, 3> velocity_ned;
  std::array<double, 4> q_ned_from_frd;
  std::array<double, 3> angular_rate_frd;
  std::array<double, 4> motor_command;
  std::array<std::uint16_t, 4> rc_aetr;
  bool crashed;
};

// The loop pushes rows; a writer thread drains them to the file
class TruthLog {
 public:
  explicit TruthLog(const std::filesystem::path& path);
  ~TruthLog();
  TruthLog(const TruthLog&) = delete;
  TruthLog& operator=(const TruthLog&) = delete;

  void push(const TruthRow& row) noexcept;
  [[nodiscard]] std::uint64_t dropped_rows() const { return dropped_; }

 private:
  void run(const std::stop_token& stop);
  void write(const TruthRow& row);

  std::ofstream file_;
  SpscQueue<TruthRow, 4096> queue_;
  std::uint64_t dropped_ = 0;
  std::jthread writer_;
};

}  // namespace fpvsim::log
