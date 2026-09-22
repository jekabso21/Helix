#pragma once

#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <fpvsim/input/joystick.hpp>
#include <fpvsim/input/mapping.hpp>

namespace fpvsim::input {

struct InputStatus {
  std::string source;
  DeviceInfo device;
  InputMapping mapping;
  std::vector<std::string> device_names;
};

// Hand-over between the I/O thread and the loop; the loop only ever try_locks
class InputControl {
 public:
  void request_mapping(const InputMapping& mapping);
  void request_device(const std::string& name_contains);
  [[nodiscard]] InputStatus status() const;

  bool take_mapping(InputMapping& out) noexcept;
  bool take_device(std::string& out) noexcept;
  bool publish_status(const InputStatus& status) noexcept;

 private:
  mutable std::mutex mutex_;
  std::optional<InputMapping> pending_mapping_;
  std::optional<std::string> pending_device_;
  InputStatus status_;
};

}  // namespace fpvsim::input
