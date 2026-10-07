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
                                 .baro_drift_pa = 0.0,
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

TEST(ImuNoiseTest, BarometerDriftIsARandomWalkOnTopOfTheBias) {
  sensors::ImuNoiseParams params = quiet();
  params.baro_drift_pa = 2.0;  // Pa/sqrt(s)
  const int runs = 200;
  const int steps = 1000;  // 1 s at 1 kHz
  double sum_sq = 0.0;
  for (int r = 0; r < runs; ++r) {
    params.seed = 5000 + r;
    sensors::ImuNoise noise(params);
    for (int i = 0; i < steps; ++i) {
      noise.apply_baro(101325.0);
    }
    sum_sq += noise.baro_drift() * noise.baro_drift();
  }
  EXPECT_NEAR(std::sqrt(sum_sq / runs), params.baro_drift_pa, 0.15 * params.baro_drift_pa);
}

namespace {

sensors::ImuSample moving(double t) {
  return sensors::ImuSample{.angular_rate_frd = Vector3d(std::sin(t), 2.0 * t, -t),
                            .specific_force_frd = Vector3d(t, -t, -fpvsim::kStandardGravityMps2)};
}

}  // namespace

TEST(ImuFaultTest, HealthyFaultsChangeNothing) {
  sensors::ImuNoiseParams p = quiet();
  p.gyro_noise_density = 0.01;
  p.accel_noise_density = 0.05;
  p.vibration_imbalance = 2e-7;
  sensors::ImuNoise plain(p);
  sensors::ImuNoise faulted(p);
  faulted.set_faults(sensors::healthy_imu_faults());
  for (int i = 0; i < 100; ++i) {
    const sensors::ImuSample a = plain.apply(moving(i * 0.001), spinning(1500.0), 1, 3, i * 0.001);
    const sensors::ImuSample b = faulted.apply(moving(i * 0.001), spinning(1500.0), 1, 3, i * 0.001);
    ASSERT_EQ(a.angular_rate_frd, b.angular_rate_frd);
    ASSERT_EQ(a.specific_force_frd, b.specific_force_frd);
  }
}

TEST(ImuFaultTest, NoiseScaleMultipliesTheWhiteNoise) {
  sensors::ImuNoiseParams p = quiet();
  p.gyro_noise_density = 0.002;
  sensors::ImuNoise noise(p);
  sensors::ImuFaults faults = sensors::healthy_imu_faults();
  faults.gyro.noise_scale = 3.0;
  noise.set_faults(faults);
  double sum2 = 0.0;
  const int n = 20000;
  for (int i = 0; i < n; ++i) {
    const double x = noise.apply(rest(), spinning(0.0), 1, 3, 0.0).angular_rate_frd.x();
    sum2 += x * x;
  }
  const double expected_sigma = 3.0 * p.gyro_noise_density * std::sqrt(p.sample_rate_hz);
  EXPECT_NEAR(std::sqrt(sum2 / n) / expected_sigma, 1.0, 0.05);
}

TEST(ImuFaultTest, BiasStepAddsOnTopOfTheSignal) {
  sensors::ImuNoise noise(quiet());
  sensors::ImuFaults faults = sensors::healthy_imu_faults();
  faults.gyro.bias_step = Vector3d(0.3, 0.0, -0.1);
  faults.accel.bias_step = Vector3d(0.0, 1.5, 0.0);
  noise.set_faults(faults);
  const sensors::ImuSample out = noise.apply(rest(), spinning(0.0), 1, 3, 0.0);
  EXPECT_DOUBLE_EQ(out.angular_rate_frd.x(), 0.3);
  EXPECT_DOUBLE_EQ(out.angular_rate_frd.z(), -0.1);
  EXPECT_DOUBLE_EQ(out.specific_force_frd.y(), 1.5);
}

TEST(ImuFaultTest, AStuckAxisHoldsItsLastValueUntilReleased) {
  sensors::ImuNoise noise(quiet());
  (void)noise.apply(moving(1.0), spinning(0.0), 1, 3, 0.0);  // gyro y = 2.0
  sensors::ImuFaults faults = sensors::healthy_imu_faults();
  faults.gyro.stuck = {false, true, false};
  noise.set_faults(faults);
  const sensors::ImuSample held = noise.apply(moving(5.0), spinning(0.0), 1, 3, 0.0);
  EXPECT_DOUBLE_EQ(held.angular_rate_frd.y(), 2.0);
  EXPECT_DOUBLE_EQ(held.angular_rate_frd.z(), -5.0);  // the other axes still move
  noise.set_faults(sensors::healthy_imu_faults());
  EXPECT_DOUBLE_EQ(noise.apply(moving(5.0), spinning(0.0), 1, 3, 0.0).angular_rate_frd.y(), 10.0);
}

TEST(ImuFaultTest, RangeScaleShrinksTheSaturation) {
  sensors::ImuNoiseParams p = quiet();
  p.gyro_range_radps = 10.0;
  sensors::ImuNoise noise(p);
  sensors::ImuFaults faults = sensors::healthy_imu_faults();
  faults.gyro.range_scale = 0.25;
  noise.set_faults(faults);
  sensors::ImuSample fast = rest();
  fast.angular_rate_frd = Vector3d(8.0, -8.0, 1.0);
  const sensors::ImuSample out = noise.apply(fast, spinning(0.0), 1, 3, 0.0);
  EXPECT_DOUBLE_EQ(out.angular_rate_frd.x(), 2.5);
  EXPECT_DOUBLE_EQ(out.angular_rate_frd.y(), -2.5);
  EXPECT_DOUBLE_EQ(out.angular_rate_frd.z(), 1.0);
}

TEST(ImuFaultTest, ADamagedPropShakesHarderInProportion) {
  sensors::ImuNoiseParams p = quiet();
  p.vibration_imbalance = 2e-7;
  sensors::ImuNoise healthy(p);
  sensors::ImuNoise damaged(p);
  sensors::ImuFaults faults = sensors::healthy_imu_faults();
  faults.imbalance_scale[0] = 10.0;
  damaged.set_faults(faults);
  for (int i = 1; i < 50; ++i) {
    const double t = i * 0.00037;
    const double a = healthy.apply(rest(), spinning(1500.0), 1, 3, t).specific_force_frd.z() +
                     fpvsim::kStandardGravityMps2;
    const double b = damaged.apply(rest(), spinning(1500.0), 1, 3, t).specific_force_frd.z() +
                     fpvsim::kStandardGravityMps2;
    ASSERT_NEAR(b, 10.0 * a, 1e-9 + 1e-9 * std::abs(a));
  }
}

TEST(ImuFaultTest, BarometerCanStickOrJump) {
  sensors::ImuNoise noise(quiet());
  EXPECT_DOUBLE_EQ(noise.apply_baro(101000.0).pressure_pa, 101000.0);
  sensors::ImuFaults faults = sensors::healthy_imu_faults();
  faults.baro_offset_pa = -250.0;
  noise.set_faults(faults);
  EXPECT_DOUBLE_EQ(noise.apply_baro(101000.0).pressure_pa, 100750.0);
  faults.baro_stuck = true;
  noise.set_faults(faults);
  EXPECT_DOUBLE_EQ(noise.apply_baro(99000.0).pressure_pa, 100750.0);
}
