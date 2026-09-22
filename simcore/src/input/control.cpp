#include <fpvsim/input/control.hpp>

namespace fpvsim::input {

void InputControl::request_mapping(const InputMapping& mapping) {
  const std::scoped_lock lock(mutex_);
  pending_mapping_ = mapping;
}

void InputControl::request_device(const std::string& name_contains) {
  const std::scoped_lock lock(mutex_);
  pending_device_ = name_contains;
}

InputStatus InputControl::status() const {
  const std::scoped_lock lock(mutex_);
  return status_;
}

bool InputControl::take_mapping(InputMapping& out) noexcept {
  const std::unique_lock lock(mutex_, std::try_to_lock);
  if (!lock.owns_lock() || !pending_mapping_) {
    return false;
  }
  out = *pending_mapping_;
  pending_mapping_.reset();
  return true;
}

bool InputControl::take_device(std::string& out) noexcept {
  const std::unique_lock lock(mutex_, std::try_to_lock);
  if (!lock.owns_lock() || !pending_device_) {
    return false;
  }
  out = *pending_device_;
  pending_device_.reset();
  return true;
}

bool InputControl::publish_status(const InputStatus& status) noexcept {
  const std::unique_lock lock(mutex_, std::try_to_lock);
  if (!lock.owns_lock()) {
    return false;
  }
  status_ = status;
  return true;
}

}  // namespace fpvsim::input
