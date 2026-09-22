#pragma once

#include <memory>
#include <string>
#include <vector>

#include <fpvsim/input/mapping.hpp>

namespace fpvsim::input {

// SDL2 joystick opened by a substring of its name; construction throws if none matches
class Joystick {
 public:
  explicit Joystick(const std::string& name_contains);
  ~Joystick();
  Joystick(const Joystick&) = delete;
  Joystick& operator=(const Joystick&) = delete;

  // Reads the current axes and buttons without blocking
  DeviceState poll() noexcept;
  [[nodiscard]] const std::string& name() const { return name_; }
  [[nodiscard]] bool connected() const noexcept;

  static std::vector<std::string> connected_names();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  std::string name_;
};

}  // namespace fpvsim::input
