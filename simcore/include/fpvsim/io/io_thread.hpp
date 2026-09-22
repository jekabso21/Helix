#pragma once

#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>

#include <nlohmann/json.hpp>

#include <fpvsim/input/control.hpp>
#include <fpvsim/sim/model_control.hpp>
#include <fpvsim/sim/snapshot.hpp>
#include <fpvsim/spsc_queue.hpp>

namespace fpvsim::io {

using SnapshotQueue = SpscQueue<sim::Snapshot, 4096>;
using CommandQueue = SpscQueue<sim::Command, 64>;
using ResultQueue = SpscQueue<sim::CommandResult, 64>;

struct IoConfig {
  std::string api_host;
  std::uint16_t api_port;
  std::string app_host;
  std::uint16_t app_port;
  std::int64_t physics_rate_hz;
  std::int64_t state_rate_hz;
  std::int64_t log_rate_hz;
  std::filesystem::path truth_csv;
  std::string input_source;
  nlohmann::json info;
};

struct IoStats {
  std::uint64_t render_states_sent = 0;
  std::uint64_t log_rows = 0;
  std::uint64_t api_requests = 0;
  std::uint64_t clients_seen = 0;
};

// Everything that is not the physics loop: RenderState, truth log, control API, telemetry
class IoThread {
 public:
  IoThread(input::InputControl& input_control, sim::ModelControl& model_control, IoConfig config,
           SnapshotQueue& snapshots, CommandQueue& commands, ResultQueue& results);
  ~IoThread();
  IoThread(const IoThread&) = delete;
  IoThread& operator=(const IoThread&) = delete;

  // Valid after stop(); joins the thread
  IoStats stop();

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
  std::jthread thread_;
};

}  // namespace fpvsim::io
