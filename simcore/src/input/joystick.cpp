#include <fpvsim/input/joystick.hpp>

#include <SDL2/SDL.h>

#include <algorithm>
#include <cctype>
#include <stdexcept>

namespace fpvsim::input {

namespace {

constexpr double kAxisFullScale = 32767.0;

std::string lower(const std::string& s) {
  std::string out = s;
  std::ranges::transform(out, out.begin(),
                         [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return out;
}

std::string name_at(int index) {
  const char* raw = SDL_JoystickNameForIndex(index);
  return raw != nullptr ? raw : "(unnamed)";
}

}  // namespace

struct JoystickManager::Impl {
  SDL_Joystick* handle = nullptr;
  std::string name;
};

JoystickManager::JoystickManager() : impl_(std::make_unique<Impl>()) {
  if (SDL_InitSubSystem(SDL_INIT_JOYSTICK) != 0) {
    throw std::runtime_error(std::string("SDL joystick init: ") + SDL_GetError());
  }
  SDL_JoystickEventState(SDL_IGNORE);
}

JoystickManager::~JoystickManager() {
  close();
  SDL_QuitSubSystem(SDL_INIT_JOYSTICK);
}

void JoystickManager::close() {
  if (impl_->handle != nullptr) {
    SDL_JoystickClose(impl_->handle);
    impl_->handle = nullptr;
  }
  impl_->name.clear();
}

std::vector<std::string> JoystickManager::device_names() {
  SDL_JoystickUpdate();
  const int count = SDL_NumJoysticks();
  std::vector<std::string> names;
  names.reserve(static_cast<std::size_t>(std::max(0, count)));
  for (int i = 0; i < count; ++i) {
    names.push_back(name_at(i));
  }
  return names;
}

bool JoystickManager::open(const std::string& name_contains) {
  close();
  SDL_JoystickUpdate();
  const std::string wanted = lower(name_contains);
  for (int i = 0; i < SDL_NumJoysticks(); ++i) {
    const std::string name = name_at(i);
    if (wanted.empty() || lower(name).find(wanted) != std::string::npos) {
      impl_->handle = SDL_JoystickOpen(i);
      if (impl_->handle == nullptr) {
        continue;
      }
      impl_->name = name;
      return true;
    }
  }
  return false;
}

DeviceInfo JoystickManager::info() const {
  const bool connected =
      impl_->handle != nullptr && SDL_JoystickGetAttached(impl_->handle) == SDL_TRUE;
  return DeviceInfo{
      .name = impl_->name,
      .connected = connected,
      .axis_count = connected ? static_cast<std::size_t>(SDL_JoystickNumAxes(impl_->handle)) : 0,
      .button_count =
          connected ? static_cast<std::size_t>(SDL_JoystickNumButtons(impl_->handle)) : 0};
}

DeviceState JoystickManager::poll() noexcept {
  DeviceState state{};
  SDL_JoystickUpdate();
  if (impl_->handle == nullptr || SDL_JoystickGetAttached(impl_->handle) != SDL_TRUE) {
    return state;
  }
  const int axes = std::min(SDL_JoystickNumAxes(impl_->handle), static_cast<int>(kMaxAxes));
  for (int i = 0; i < axes; ++i) {
    state.axes[static_cast<std::size_t>(i)] =
        std::clamp(SDL_JoystickGetAxis(impl_->handle, i) / kAxisFullScale, -1.0, 1.0);
  }
  const int buttons =
      std::min(SDL_JoystickNumButtons(impl_->handle), static_cast<int>(kMaxButtons));
  for (int i = 0; i < buttons; ++i) {
    state.buttons[static_cast<std::size_t>(i)] = SDL_JoystickGetButton(impl_->handle, i) != 0;
  }
  return state;
}

}  // namespace fpvsim::input
