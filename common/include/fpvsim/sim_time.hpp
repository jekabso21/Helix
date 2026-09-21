#pragma once

#include <compare>
#include <cstdint>
#include <optional>

namespace fpvsim {

inline constexpr std::int64_t kNanosecondsPerSecond = 1'000'000'000;

// Integer nanoseconds since run start
struct SimTime {
  std::int64_t ns = 0;

  constexpr auto operator<=>(const SimTime&) const = default;

  constexpr SimTime& operator+=(SimTime rhs) {
    ns += rhs.ns;
    return *this;
  }
  constexpr SimTime& operator-=(SimTime rhs) {
    ns -= rhs.ns;
    return *this;
  }
};

constexpr SimTime operator+(SimTime lhs, SimTime rhs) { return SimTime{lhs.ns + rhs.ns}; }
constexpr SimTime operator-(SimTime lhs, SimTime rhs) { return SimTime{lhs.ns - rhs.ns}; }
constexpr SimTime operator*(SimTime lhs, std::int64_t rhs) { return SimTime{lhs.ns * rhs}; }

// For interfaces that need seconds (FDM timestamp); never accumulate time in floating point
constexpr double to_seconds(SimTime t) {
  return static_cast<double>(t.ns) / static_cast<double>(kNanosecondsPerSecond);
}

// Empty unless rate_hz is positive and divides one second into whole nanoseconds
constexpr std::optional<SimTime> step_from_rate_hz(std::int64_t rate_hz) {
  if (rate_hz <= 0 || kNanosecondsPerSecond % rate_hz != 0) {
    return std::nullopt;
  }
  return SimTime{kNanosecondsPerSecond / rate_hz};
}

}  // namespace fpvsim
