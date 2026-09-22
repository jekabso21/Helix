#pragma once

#include <mutex>
#include <optional>

#include <fpvsim/sim/vehicle.hpp>

namespace fpvsim::sim {

// Hand-over of a reloaded drone model from the I/O thread to the loop; the loop only try_locks
class ModelControl {
 public:
  void request(const VehicleParams& params);
  bool take(VehicleParams& out) noexcept;

 private:
  std::mutex mutex_;
  std::optional<VehicleParams> pending_;
};

}  // namespace fpvsim::sim
