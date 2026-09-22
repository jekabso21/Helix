#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <random>

#include <Eigen/Core>

#include <fpvsim/physics/motor.hpp>
#include <fpvsim/physics/propulsion.hpp>
#include <fpvsim/sensors/imu.hpp>

namespace fpvsim::sensors {

struct ImuNoiseParams {
  double sample_rate_hz;
  double gyro_noise_density;    // rad/s/sqrt(Hz)
  double gyro_bias_walk;        // rad/s^2/sqrt(Hz)
  double gyro_range_radps;      // saturation, 0 disables
  double accel_noise_density;   // m/s^2/sqrt(Hz)
  double accel_bias_walk;       // m/s^3/sqrt(Hz)
  double accel_range_mps2;      // saturation, 0 disables
  double vibration_imbalance;   // m/s^2 per (rad/s)^2 per motor at the rotation frequency
  double vibration_harmonic2;   // fraction of the fundamental at twice the rotation frequency
  double vibration_blade_pass;  // fraction of the fundamental at blades x rotation frequency
  double vibration_gyro_gain;   // rad/s of gyro vibration per m/s^2 of accel vibration
  double baro_noise_pa;         // white noise standard deviation per sample
  double baro_bias_pa;
  std::uint64_t seed;
};

struct BaroSample {
  double pressure_pa;
};

// Seeded, allocation-free noise on top of the ideal IMU; deterministic for a given seed
class ImuNoise {
 public:
  explicit ImuNoise(const ImuNoiseParams& params);

  ImuSample apply(const ImuSample& ideal,
                  const std::array<physics::MotorOutput, physics::kMaxMotors>& motors,
                  std::size_t motor_count, int blades, double time_s);
  BaroSample apply_baro(double true_pressure_pa);

  [[nodiscard]] const Eigen::Vector3d& gyro_bias() const { return gyro_bias_; }
  [[nodiscard]] const Eigen::Vector3d& accel_bias() const { return accel_bias_; }

 private:
  Eigen::Vector3d vibration(const std::array<physics::MotorOutput, physics::kMaxMotors>& motors,
                            std::size_t motor_count, int blades, double time_s) const;
  double gaussian();

  ImuNoiseParams params_;
  std::mt19937_64 rng_;
  std::normal_distribution<double> normal_;
  Eigen::Vector3d gyro_bias_;
  Eigen::Vector3d accel_bias_;
  std::array<std::array<double, 3>, physics::kMaxMotors> phase_;
  std::array<Eigen::Vector3d, physics::kMaxMotors> axis_weight_;
};

}  // namespace fpvsim::sensors
