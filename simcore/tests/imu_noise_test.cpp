#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

#include <fpvsim/constants.hpp>
#include <fpvsim/sensors/imu_noise.hpp>

namespace sensors = fpvsim::sensors;
namespace physics = fpvsim::physics;
using Eigen::Vector3d;

namespace {

sensors::ImuNoiseParams quiet() {
  return sensors::ImuNoiseParams{.sample_rate_hz = 1000.0,
                                 .gyro_noise_density = 0.0,
                                 .gyro_bias_walk = 0.0,
                                 .gyro_range_radps = 0.0,
                                 .accel_noise_density = 0.0,
                                 .accel_bias_walk = 0.0,
                                 .accel_range_mps2 = 0.0,
                                 .vibration_imbalance = 0.0,
                                 .vibration_harmonic2 = 0.0,
                                 .vibration_blade_pass = 0.0,
                                 .vibration_gyro_gain = 0.0,
                                 .baro_noise_pa = 0.0,
                                 .baro_bias_pa = 0.0,
                                 .seed = 42};
}

sensors::ImuSample rest() {
  return sensors::ImuSample{
      .angular_rate_frd = Vector3d::Zero(),
      .specific_force_frd = Vector3d(0.0, 0.0, -fpvsim::kStandardGravityMps2)};
}

std::array<physics::MotorOutput, physics::kMaxMotors> spinning(double speed) {
  std::array<physics::MotorOutput, physics::kMaxMotors> motors{};
  motors[0].speed_radps = speed;
  return motors;
}

}  // namespace

TEST(ImuNoiseTest, ZeroParametersPassTheIdealSampleThrough) {
  sensors::ImuNoise noise(quiet());
  const sensors::ImuSample out = noise.apply(rest(), spinning(2000.0), 1, 3, 0.5);
  EXPECT_TRUE(out.angular_rate_frd.isZero());
  EXPECT_NEAR(out.specific_force_frd.z(), -fpvsim::kStandardGravityMps2, 1e-12);
  EXPECT_DOUBLE_EQ(noise.apply_baro(101325.0).pressure_pa, 101325.0);
}

TEST(ImuNoiseTest, WhiteNoiseHasTheDensityTimesRootSampleRate) {
  sensors::ImuNoiseParams params = quiet();
  params.gyro_noise_density = 0.001;  // rad/s/sqrt(Hz)
  params.accel_noise_density = 0.02;  // m/s^2/sqrt(Hz)
  sensors::ImuNoise noise(params);
  const int n = 20000;
  double gyro_sq = 0.0;
  double accel_sq = 0.0;
  double accel_sum = 0.0;
  for (int i = 0; i < n; ++i) {
    const sensors::ImuSample s = noise.apply(rest(), {}, 0, 3, i * 0.001);
    gyro_sq += s.angular_rate_frd.x() * s.angular_rate_frd.x();
    const double az = s.specific_force_frd.z() + fpvsim::kStandardGravityMps2;
    accel_sq += az * az;
    accel_sum += az;
  }
  const double gyro_sigma = params.gyro_noise_density * std::sqrt(params.sample_rate_hz);
  const double accel_sigma = params.accel_noise_density * std::sqrt(params.sample_rate_hz);
  EXPECT_NEAR(std::sqrt(gyro_sq / n), gyro_sigma, 0.03 * gyro_sigma);
  EXPECT_NEAR(std::sqrt(accel_sq / n), accel_sigma, 0.03 * accel_sigma);
  EXPECT_NEAR(accel_sum / n, 0.0, 0.03 * accel_sigma);
}

TEST(ImuNoiseTest, SameSeedSameSequence) {
  sensors::ImuNoiseParams params = quiet();
  params.gyro_noise_density = 0.001;
  params.gyro_bias_walk = 0.0001;
  sensors::ImuNoise a(params);
  sensors::ImuNoise b(params);
  for (int i = 0; i < 100; ++i) {
    EXPECT_EQ(a.apply(rest(), {}, 0, 3, 0.0).angular_rate_frd,
              b.apply(rest(), {}, 0, 3, 0.0).angular_rate_frd);
  }
  params.seed = 7;
  sensors::ImuNoise c(params);
  EXPECT_NE(a.apply(rest(), {}, 0, 3, 0.0).angular_rate_frd,
            c.apply(rest(), {}, 0, 3, 0.0).angular_rate_frd);
}

TEST(ImuNoiseTest, BiasRandomWalkGrowsWithRootTime) {
  sensors::ImuNoiseParams params = quiet();
  params.gyro_bias_walk = 0.01;
  const int runs = 200;
  const int steps = 1000;  // 1 s
  double sum_sq = 0.0;
  for (int r = 0; r < runs; ++r) {
    params.seed = 1000 + r;
    sensors::ImuNoise noise(params);
    for (int i = 0; i < steps; ++i) {
      noise.apply(rest(), {}, 0, 3, 0.0);
    }
    sum_sq += noise.gyro_bias().x() * noise.gyro_bias().x();
  }
  EXPECT_NEAR(std::sqrt(sum_sq / runs), params.gyro_bias_walk * std::sqrt(1.0),
              0.15 * params.gyro_bias_walk);
}

TEST(ImuNoiseTest, VibrationSitsAtTheRotationFrequencyWithImbalanceAmplitude) {
  sensors::ImuNoiseParams params = quiet();
  params.vibration_imbalance = 1e-6;
  sensors::ImuNoise noise(params);
  const double w = 2000.0;
  const double amplitude = params.vibration_imbalance * w * w;  // 4 m/s^2 on the unit-weight axis
  const int n = 4000;
  double sum_sq = 0.0;
  double in_phase = 0.0;
  double quadrature = 0.0;
  for (int i = 0; i < n; ++i) {
    const double t = i * 0.001;
    const double az = noise.apply(rest(), spinning(w), 1, 3, t).specific_force_frd.z() +
                      fpvsim::kStandardGravityMps2;
    sum_sq += az * az;
    in_phase += az * std::sin(w * t);
    quadrature += az * std::cos(w * t);
  }
  EXPECT_NEAR(std::sqrt(sum_sq / n), amplitude / std::sqrt(2.0), 0.02 * amplitude);
  // a single tone at w: projecting onto sin and cos at w recovers the amplitude
  EXPECT_NEAR(2.0 * std::hypot(in_phase, quadrature) / n, amplitude, 0.02 * amplitude);
}

TEST(ImuNoiseTest, SaturationClampsToTheRange) {
  sensors::ImuNoiseParams params = quiet();
  params.gyro_range_radps = 10.0;
  params.accel_range_mps2 = 5.0;
  sensors::ImuNoise noise(params);
  const sensors::ImuSample big{
      .angular_rate_frd = Vector3d(50.0, -50.0, 1.0),
      .specific_force_frd = Vector3d(0.0, 0.0, -fpvsim::kStandardGravityMps2)};
  const sensors::ImuSample out = noise.apply(big, {}, 0, 3, 0.0);
  EXPECT_EQ(out.angular_rate_frd, Vector3d(10.0, -10.0, 1.0));
  EXPECT_DOUBLE_EQ(out.specific_force_frd.z(), -5.0);
}

TEST(ImuNoiseTest, BarometerAddsBiasAndNoise) {
  sensors::ImuNoiseParams params = quiet();
  params.baro_bias_pa = 30.0;
  params.baro_noise_pa = 5.0;
  sensors::ImuNoise noise(params);
  double sum = 0.0;
  double sum_sq = 0.0;
  const int n = 20000;
  for (int i = 0; i < n; ++i) {
    const double d = noise.apply_baro(101325.0).pressure_pa - 101325.0;
    sum += d;
    sum_sq += d * d;
  }
  const double mean = sum / n;
  EXPECT_NEAR(mean, 30.0, 0.2);
  EXPECT_NEAR(std::sqrt(sum_sq / n - mean * mean), 5.0, 0.2);
}
