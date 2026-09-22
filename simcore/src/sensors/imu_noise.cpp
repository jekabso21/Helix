#include <fpvsim/sensors/imu_noise.hpp>

#include <cmath>
#include <numbers>

namespace fpvsim::sensors {

namespace {

Eigen::Vector3d saturate(const Eigen::Vector3d& v, double range) {
  if (range <= 0.0) {
    return v;
  }
  return v.cwiseMax(-range).cwiseMin(range);
}

}  // namespace

ImuNoise::ImuNoise(const ImuNoiseParams& params)
    : params_(params),
      rng_(params.seed),
      normal_(0.0, 1.0),
      gyro_bias_(Eigen::Vector3d::Zero()),
      accel_bias_(Eigen::Vector3d::Zero()),
      phase_{},
      axis_weight_{} {
  std::uniform_real_distribution<double> phase(0.0, 2.0 * std::numbers::pi);
  for (std::size_t i = 0; i < physics::kMaxMotors; ++i) {
    for (double& p : phase_[i]) {
      p = phase(rng_);
    }
    // mostly vertical with a random in-plane share, as a bent prop shakes a frame
    axis_weight_[i] = Eigen::Vector3d(gaussian() * 0.3, gaussian() * 0.3, 1.0);
  }
}

double ImuNoise::gaussian() { return normal_(rng_); }

Eigen::Vector3d ImuNoise::vibration(
    const std::array<physics::MotorOutput, physics::kMaxMotors>& motors, std::size_t motor_count,
    int blades, double time_s) const {
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  if (params_.vibration_imbalance <= 0.0) {
    return sum;
  }
  for (std::size_t i = 0; i < motor_count; ++i) {
    const double w = motors[i].speed_radps;
    const double amplitude = params_.vibration_imbalance * w * w;
    const double wt = w * time_s;
    const double signal = std::sin(wt + phase_[i][0]) +
                          params_.vibration_harmonic2 * std::sin(2.0 * wt + phase_[i][1]) +
                          params_.vibration_blade_pass * std::sin(blades * wt + phase_[i][2]);
    sum += amplitude * signal * axis_weight_[i];
  }
  return sum;
}

ImuSample ImuNoise::apply(const ImuSample& ideal,
                          const std::array<physics::MotorOutput, physics::kMaxMotors>& motors,
                          std::size_t motor_count, int blades, double time_s) {
  const double dt = 1.0 / params_.sample_rate_hz;
  const double sqrt_dt = std::sqrt(dt);
  const double gyro_sigma = params_.gyro_noise_density / sqrt_dt;  // ND * sqrt(fs)
  const double accel_sigma = params_.accel_noise_density / sqrt_dt;
  for (int k = 0; k < 3; ++k) {
    gyro_bias_[k] += params_.gyro_bias_walk * sqrt_dt * gaussian();
    accel_bias_[k] += params_.accel_bias_walk * sqrt_dt * gaussian();
  }
  const Eigen::Vector3d shake = vibration(motors, motor_count, blades, time_s);
  Eigen::Vector3d gyro = ideal.angular_rate_frd + gyro_bias_ + params_.vibration_gyro_gain * shake;
  Eigen::Vector3d accel = ideal.specific_force_frd + accel_bias_ + shake;
  for (int k = 0; k < 3; ++k) {
    gyro[k] += gyro_sigma * gaussian();
    accel[k] += accel_sigma * gaussian();
  }
  return ImuSample{.angular_rate_frd = saturate(gyro, params_.gyro_range_radps),
                   .specific_force_frd = saturate(accel, params_.accel_range_mps2)};
}

BaroSample ImuNoise::apply_baro(double true_pressure_pa) {
  return BaroSample{.pressure_pa = true_pressure_pa + params_.baro_bias_pa +
                                   params_.baro_noise_pa * gaussian()};
}

}  // namespace fpvsim::sensors
