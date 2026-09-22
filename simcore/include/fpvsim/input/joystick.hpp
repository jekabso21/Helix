#pragma once

#include <memory>
#include <string>
#include <vector>

#include <fpvsim/input/mapping.hpp>

namespace fpvsim::input {

struct DeviceInfo {
  std::string name;
  bool connected;
  std::size_t axis_count;
  std::size_t button_count;
};

// SDL2 joystick access from one thread: open by a substring of the name, poll without blocking
class JoystickManager {
 public:
  JoystickManager();
  ~JoystickManager();
  JoystickManager(const JoystickManager&) = delete;
  JoystickManager& operator=(const JoystickManager&) = delete;

  // Closes any open device and opens the first whose name contains the text; false if none
  bool open(const std::string& name_contains);
  void close();
  [[nodiscard]] DeviceInfo info() const;
  [[nodiscard]] static std::vector<std::string> device_names();

  // Neutral state when no device is open or it was unplugged
  DeviceState poll() noexcept;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace fpvsim::input
