#include <fpvsim/sim/failures.hpp>

#include <cmath>

namespace fpvsim::sim {

namespace {

bool in_effect(const ActiveFailure& failure, double time_s) {
  return time_s >= failure.start_s && (failure.end_s < 0.0 || time_s < failure.end_s);
}

sensors::SensorFault& sensor_of(sensors::ImuFaults& imu, Sensor sensor) {
  return sensor == Sensor::kGyro ? imu.gyro : imu.accel;
}

void apply(FailureModifiers& m, const FailureSpec& s) {
  const std::size_t motor = s.motor < physics::kMaxMotors ? s.motor : 0;
  MotorFault& mf = m.motors[motor];
  switch (s.type) {
    case FailureType::kMotorOut:
      mf.output_gain = 0.0;
      break;
    case FailureType::kMotorDegraded:
      mf.output_gain *= s.value;
      break;
    case FailureType::kEscDesync:
      mf.desync_period_s = s.value;
      mf.desync_dropout_s = s.value2;
      break;
    case FailureType::kPropDamage:
      // a chipped prop moves less air and needs less torque for it
      mf.thrust_scale *= 1.0 - s.value;
      mf.torque_scale *= 1.0 - s.value;
      m.imu.imbalance_scale[motor] *= s.value2;
      break;
    case FailureType::kBatteryWeakCell:
      m.weak_cell_drop_v += s.value;
      break;
    case FailureType::kBatteryHighResistance:
      m.cell_resistance_scale *= s.value;
      m.connector_resistance_add_ohm += s.value2;
      break;
    case FailureType::kImuNoise:
      sensor_of(m.imu, s.sensor).noise_scale *= s.value;
      break;
    case FailureType::kImuBias:
      for (std::uint8_t k = 0; k < 3; ++k) {
        if (s.axis == kAllAxes || s.axis == k) {
          sensor_of(m.imu, s.sensor).bias_step[k] += s.value;
        }
      }
      break;
    case FailureType::kImuStuck:
      for (std::uint8_t k = 0; k < 3; ++k) {
        if (s.axis == kAllAxes || s.axis == k) {
          sensor_of(m.imu, s.sensor).stuck[k] = true;
        }
      }
      break;
    case FailureType::kImuSaturation:
      sensor_of(m.imu, s.sensor).range_scale *= s.value;
      break;
    case FailureType::kBaroStuck:
      m.imu.baro_stuck = true;
      break;
    case FailureType::kBaroOffset:
      m.imu.baro_offset_pa += s.value;
      break;
  }
}

}  // namespace

FailureModifiers healthy_modifiers() {
  FailureModifiers m{.motors = {},
                     .cell_resistance_scale = 1.0,
                     .connector_resistance_add_ohm = 0.0,
                     .weak_cell_drop_v = 0.0,
                     .imu = sensors::healthy_imu_faults()};
  m.motors.fill(MotorFault{.output_gain = 1.0,
                           .thrust_scale = 1.0,
                           .torque_scale = 1.0,
                           .desync_period_s = 0.0,
                           .desync_dropout_s = 0.0});
  return m;
}

FailureModifiers modifiers_from(std::span<const ActiveFailure> failures, double time_s) {
  FailureModifiers m = healthy_modifiers();
  for (const ActiveFailure& failure : failures) {
    if (in_effect(failure, time_s)) {
      apply(m, failure.spec);
    }
  }
  return m;
}

bool desync_dropped_out(const MotorFault& fault, double time_s) {
  if (fault.desync_period_s <= 0.0) {
    return false;
  }
  return std::fmod(time_s, fault.desync_period_s) < fault.desync_dropout_s;
}

bool FailureSet::add(std::uint32_t id, const FailureSpec& spec, double start_s, double end_s) {
  if (count_ >= kMaxFailures) {
    return false;
  }
  failures_[count_++] = ActiveFailure{.id = id, .spec = spec, .start_s = start_s, .end_s = end_s};
  dirty_ = true;
  return true;
}

bool FailureSet::remove(std::uint32_t id) {
  for (std::size_t i = 0; i < count_; ++i) {
    if (failures_[i].id == id) {
      failures_[i] = failures_[--count_];
      dirty_ = true;
      return true;
    }
  }
  return false;
}

void FailureSet::clear() {
  count_ = 0;
  dirty_ = true;
}

bool FailureSet::update(double time_s) {
  std::size_t kept = 0;
  std::size_t effective = 0;
  for (std::size_t i = 0; i < count_; ++i) {
    const ActiveFailure& f = failures_[i];
    if (f.end_s >= 0.0 && time_s >= f.end_s) {
      dirty_ = true;
      continue;
    }
    if (in_effect(f, time_s)) {
      ++effective;
    }
    failures_[kept++] = f;
  }
  count_ = kept;
  const bool changed = dirty_ || effective != in_effect_;
  in_effect_ = effective;
  dirty_ = false;
  return changed;
}

}  // namespace fpvsim::sim
