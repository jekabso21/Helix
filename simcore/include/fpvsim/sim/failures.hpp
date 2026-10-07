#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include <fpvsim/physics/propulsion.hpp>
#include <fpvsim/sensors/imu_noise.hpp>

namespace fpvsim::sim {

inline constexpr std::size_t kMaxFailures = 16;
inline constexpr std::uint8_t kAllAxes = 3;

enum class FailureType : std::uint8_t {
  kMotorOut,               // motor
  kMotorDegraded,          // motor, value = output gain 0..1
  kEscDesync,              // motor, value = period s, value2 = dropout s per period
  kPropDamage,             // motor, value = thrust loss 0..1, value2 = vibration multiplier
  kBatteryWeakCell,        // value = drop of one cell, V
  kBatteryHighResistance,  // value = cell resistance multiplier, value2 = extra connector ohm
  kImuNoise,               // sensor, value = noise multiplier
  kImuBias,                // sensor, axis, value = step in rad/s (gyro) or m/s^2 (accel)
  kImuStuck,               // sensor, axis
  kImuSaturation,          // sensor, value = range multiplier 0..1
  kBaroStuck,
  kBaroOffset,             // value = offset, Pa
};

enum class Sensor : std::uint8_t { kGyro, kAccel };

// What a request asks for; trivially copyable so it can ride in a Command
struct FailureSpec {
  FailureType type;
  std::uint8_t motor;  // 0-based
  Sensor sensor;
  std::uint8_t axis;   // 0..2, or kAllAxes
  double value;
  double value2;
};

struct ActiveFailure {
  std::uint32_t id;
  FailureSpec spec;
  double start_s;
  double end_s;  // negative: until cleared
};

struct MotorFault {
  double output_gain;
  double thrust_scale;
  double torque_scale;
  double desync_period_s;  // 0: no desync pattern
  double desync_dropout_s;
};

struct FailureModifiers {
  std::array<MotorFault, physics::kMaxMotors> motors;
  double cell_resistance_scale;
  double connector_resistance_add_ohm;
  double weak_cell_drop_v;
  sensors::ImuFaults imu;
};

FailureModifiers healthy_modifiers();
// Combines the failures in effect at time_s: gains and scales multiply, offsets add
FailureModifiers modifiers_from(std::span<const ActiveFailure> failures, double time_s);
// True while a desync pattern has the ESC dropped out at time_s
bool desync_dropped_out(const MotorFault& fault, double time_s);

// The failures injected into a run, scheduled or live; fixed capacity, no heap
class FailureSet {
 public:
  // false when kMaxFailures are already held
  bool add(std::uint32_t id, const FailureSpec& spec, double start_s, double end_s);
  // false for an unknown id
  bool remove(std::uint32_t id);
  void clear();
  // Drops failures whose end has passed; true when the failures in effect changed since the last
  // call (one started or ended), so the caller knows to recompute its modifiers
  bool update(double time_s);

  [[nodiscard]] std::span<const ActiveFailure> held() const {
    return {failures_.data(), count_};
  }
  [[nodiscard]] FailureModifiers modifiers(double time_s) const {
    return modifiers_from(held(), time_s);
  }

 private:
  std::array<ActiveFailure, kMaxFailures> failures_{};
  std::size_t count_ = 0;
  std::size_t in_effect_ = 0;
  bool dirty_ = true;
};

}  // namespace fpvsim::sim
