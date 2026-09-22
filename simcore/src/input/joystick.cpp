#include <fpvsim/input/joystick.hpp>

#include <SDL2/SDL.h>

#include <algorithm>
#include <stdexcept>

namespace fpvsim::input {

namespace {

constexpr double kAxisFullScale = 32767.0;

struct SdlSubsystem {
  SdlSubsystem() {
    if (SDL_InitSubSystem(SDL_INIT_JOYSTICK) != 0) {
      throw std::runtime_error(std::string("SDL joystick init: ") + SDL_GetError());
    }
    SDL_JoystickEventState(SDL_IGNORE);
  }
  ~SdlSubsystem() { SDL_QuitSubSystem(SDL_INIT_JOYSTICK); }
};

bool contains_ignoring_case(const std::string& text, const std::string& part) {
  const auto lower = [](const std::string& s) {
    std::string out = s;
    std::ranges::transform(out, out.begin(), [](unsigned char c) { return std::tolower(c); });
    return out;
  };
  return lower(text).find(lower(part)) != std::string::npos;
}

}  // namespace

struct Joystick::Impl {
  SdlSubsystem sdl;
  SDL_Joystick* handle = nullptr;
};

std::vector<std::string> Joystick::connected_names() {
  SdlSubsystem sdl;
  std::vector<std::string> names;
  for (int i = 0; i < SDL_NumJoysticks(); ++i) {
    const char* name = SDL_JoystickNameForIndex(i);
    names.emplace_back(name != nullptr ? name : "(unnamed)");
  }
  return names;
}

Joystick::Joystick(const std::string& name_contains) : impl_(std::make_unique<Impl>()) {
  std::string seen;
  for (int i = 0; i < SDL_NumJoysticks(); ++i) {
    const char* raw = SDL_JoystickNameForIndex(i);
    const std::string name = raw != nullptr ? raw : "(unnamed)";
    seen += (seen.empty() ? "" : ", ") + name;
    if (contains_ignoring_case(name, name_contains)) {
      impl_->handle = SDL_JoystickOpen(i);
      if (impl_->handle == nullptr) {
        throw std::runtime_error("cannot open joystick '" + name + "': " + SDL_GetError());
      }
      name_ = name;
      return;
    }
  }
  throw std::runtime_error("no joystick name contains '" + name_contains +
                           "'; connected: " + (seen.empty() ? "none" : seen));
}

Joystick::~Joystick() {
  if (impl_->handle != nullptr) {
    SDL_JoystickClose(impl_->handle);
  }
}

bool Joystick::connected() const noexcept {
  return impl_->handle != nullptr && SDL_JoystickGetAttached(impl_->handle) == SDL_TRUE;
}

DeviceState Joystick::poll() noexcept {
  DeviceState state{};
  SDL_JoystickUpdate();
  if (impl_->handle == nullptr) {
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
