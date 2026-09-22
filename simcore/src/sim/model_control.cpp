#include <fpvsim/sim/model_control.hpp>

namespace fpvsim::sim {

void ModelControl::request(const VehicleParams& params) {
  const std::scoped_lock lock(mutex_);
  pending_ = params;
}

bool ModelControl::take(VehicleParams& out) noexcept {
  const std::unique_lock lock(mutex_, std::try_to_lock);
  if (!lock.owns_lock() || !pending_) {
    return false;
  }
  out = *pending_;
  pending_.reset();
  return true;
}

}  // namespace fpvsim::sim
