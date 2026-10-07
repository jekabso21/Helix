#include <gtest/gtest.h>

#include <cmath>
#include <numbers>
#include <vector>

#include <fpvsim/env/wind.hpp>

namespace env = fpvsim::env;

namespace {

constexpr double kFeet = 0.3048;
constexpr double kKnots = 1852.0 / 3600.0;

env::WindParams calm(std::uint64_t seed = 7) {
  return env::WindParams{.mean_speed_mps = 0.0,
                         .mean_from_rad = 0.0,
                         .profile = {.log_law = false, .roughness_m = 0.1, .reference_height_m = 10.0},
                         .turbulence_w20_mps = 0.0,
                         .seed = seed};
}

double deg(double d) { return d * std::numbers::pi / 180.0; }

}  // namespace

TEST(WindTest, MeanWindBlowsAwayFromWhereItComesFrom) {
  // a westerly (from 270 deg) moves air east
  const Eigen::Vector3d westerly = env::wind_from(5.0, deg(270.0));
  EXPECT_NEAR(westerly.x(), 0.0, 1e-12);
  EXPECT_NEAR(westerly.y(), 5.0, 1e-12);
  EXPECT_EQ(westerly.z(), 0.0);
  const Eigen::Vector3d northerly = env::wind_from(3.0, 0.0);
  EXPECT_NEAR(northerly.x(), -3.0, 1e-12);
  EXPECT_NEAR(northerly.y(), 0.0, 1e-12);
}

TEST(WindTest, GustFollowsOneMinusCosine) {
  const env::Gust gust{.start_s = 10.0, .duration_s = 2.0, .amplitude_mps = 4.0, .from_rad = 0.0};
  EXPECT_EQ(env::gust_speed(gust, 9.0), 0.0);
  EXPECT_NEAR(env::gust_speed(gust, 10.0), 0.0, 1e-12);
  EXPECT_NEAR(env::gust_speed(gust, 10.5), 2.0, 1e-12);  // (A/2)(1 - cos(pi/2))
  EXPECT_NEAR(env::gust_speed(gust, 11.0), 4.0, 1e-12);  // the peak is the amplitude
  EXPECT_NEAR(env::gust_speed(gust, 12.0), 0.0, 1e-12);
  EXPECT_EQ(env::gust_speed(gust, 12.5), 0.0);
}

TEST(WindTest, LogProfileIsOneAtTheReferenceHeight) {
  const env::WindProfile log{.log_law = true, .roughness_m = 0.1, .reference_height_m = 10.0};
  EXPECT_NEAR(env::profile_factor(log, 10.0), 1.0, 1e-12);
  EXPECT_NEAR(env::profile_factor(log, 1.0), std::log(10.0) / std::log(100.0), 1e-12);
  EXPECT_EQ(env::profile_factor(log, 0.05), 0.0);  // below the roughness length
  const env::WindProfile flat{.log_law = false, .roughness_m = 0.1, .reference_height_m = 10.0};
  EXPECT_EQ(env::profile_factor(flat, 0.5), 1.0);
}

TEST(WindTest, DrydenScalesFollowMilF8785cLowAltitude) {
  const double h_ft = 50.0;
  const double w20 = 15.0 * kKnots;  // light turbulence
  const env::DrydenScales s = env::dryden_low_altitude(w20, h_ft * kFeet);
  const double sigma_w = 0.1 * w20;
  EXPECT_NEAR(s.sigma_vertical_mps, sigma_w, 1e-12);
  EXPECT_NEAR(s.sigma_horizontal_mps, sigma_w / std::pow(0.177 + 0.000823 * h_ft, 0.4), 1e-12);
  EXPECT_NEAR(s.length_vertical_m, h_ft * kFeet, 1e-9);
  EXPECT_NEAR(s.length_horizontal_m, h_ft / std::pow(0.177 + 0.000823 * h_ft, 1.2) * kFeet, 1e-9);
  // below 10 ft the standard's scales are held at their 10 ft values
  const env::DrydenScales low = env::dryden_low_altitude(w20, 0.2);
  EXPECT_NEAR(low.length_vertical_m, 10.0 * kFeet, 1e-9);
}

TEST(WindTest, CalmAirHasOnlyTheMeanWind) {
  env::WindParams p = calm();
  p.mean_speed_mps = 6.0;
  p.mean_from_rad = deg(90.0);
  env::Wind wind(p);
  for (int i = 0; i < 1000; ++i) {
    const Eigen::Vector3d w = wind.step(i * 0.001, 0.001, 5.0, 6.0);
    ASSERT_NEAR(w.x(), 0.0, 1e-12);
    ASSERT_NEAR(w.y(), -6.0, 1e-12);
    ASSERT_EQ(w.z(), 0.0);
  }
}

// Each turbulence component is a first-order Gauss-Markov process discretized exactly: its
// stationary variance is sigma^2 and it decorrelates to 1/e after L / V seconds.
TEST(WindTest, TurbulenceHasTheDrydenVarianceAndCorrelationTime) {
  env::WindParams p = calm(42);
  p.turbulence_w20_mps = 30.0 * kKnots;  // moderate
  env::Wind wind(p);
  const double height = 15.0;
  const double speed = 10.0;
  const double dt = 0.01;
  const env::DrydenScales s = env::dryden_low_altitude(p.turbulence_w20_mps, height);
  const int steps = 400000;  // 4000 s
  std::vector<double> w(steps);
  double sum_u2 = 0.0;
  for (int i = 0; i < steps; ++i) {
    const Eigen::Vector3d v = wind.step(i * dt, dt, height, speed);
    w[static_cast<std::size_t>(i)] = v.z();
    sum_u2 += v.head<2>().squaredNorm() / 2.0;
  }
  double sum_w2 = 0.0;
  for (const double x : w) {
    sum_w2 += x * x;
  }
  const double var_w = sum_w2 / steps;
  EXPECT_NEAR(var_w / (s.sigma_vertical_mps * s.sigma_vertical_mps), 1.0, 0.1);
  EXPECT_NEAR(sum_u2 / steps / (s.sigma_horizontal_mps * s.sigma_horizontal_mps), 1.0, 0.25);
  const auto lag = static_cast<std::size_t>(std::lround(s.length_vertical_m / speed / dt));
  double cov = 0.0;
  for (std::size_t i = 0; i + lag < w.size(); ++i) {
    cov += w[i] * w[i + lag];
  }
  cov /= static_cast<double>(w.size() - lag);
  EXPECT_NEAR(cov / var_w, std::exp(-1.0), 0.08);
}

TEST(WindTest, TheSameSeedGivesTheSameWind) {
  env::WindParams p = calm(9);
  p.turbulence_w20_mps = 8.0;
  env::Wind a(p);
  env::Wind b(p);
  p.seed = 10;
  env::Wind c(p);
  bool differs = false;
  for (int i = 0; i < 500; ++i) {
    const Eigen::Vector3d wa = a.step(i * 0.001, 0.001, 5.0, 3.0);
    const Eigen::Vector3d wb = b.step(i * 0.001, 0.001, 5.0, 3.0);
    ASSERT_EQ(wa, wb);
    differs = differs || wa != c.step(i * 0.001, 0.001, 5.0, 3.0);
  }
  EXPECT_TRUE(differs);
}

TEST(WindTest, GustsAddAlongTheirOwnDirectionAndExpire) {
  env::WindParams p = calm();
  p.mean_speed_mps = 2.0;
  p.mean_from_rad = deg(270.0);
  env::Wind wind(p);
  ASSERT_TRUE(wind.add_gust({.start_s = 1.0, .duration_s = 2.0, .amplitude_mps = 6.0, .from_rad = 0.0}));
  const Eigen::Vector3d peak = wind.step(2.0, 0.001, 5.0, 2.0);
  EXPECT_NEAR(peak.x(), -6.0, 1e-9);  // a northerly gust pushes south
  EXPECT_NEAR(peak.y(), 2.0, 1e-9);
  const Eigen::Vector3d after = wind.step(3.5, 0.001, 5.0, 2.0);
  EXPECT_NEAR(after.x(), 0.0, 1e-9);
  EXPECT_EQ(wind.pending_gusts(), 0U);
  for (std::size_t i = 0; i < env::kMaxGusts; ++i) {
    EXPECT_TRUE(wind.add_gust({.start_s = 10.0, .duration_s = 1.0, .amplitude_mps = 1.0, .from_rad = 0.0}));
  }
  EXPECT_FALSE(wind.add_gust({.start_s = 10.0, .duration_s = 1.0, .amplitude_mps = 1.0, .from_rad = 0.0}));
}

TEST(WindTest, MeanAndTurbulenceCanChangeLive) {
  env::Wind wind(calm());
  EXPECT_NEAR(wind.step(0.0, 0.001, 5.0, 0.0).norm(), 0.0, 1e-12);
  wind.set_mean(4.0, 0.0);
  EXPECT_NEAR(wind.step(0.001, 0.001, 5.0, 4.0).x(), -4.0, 1e-12);
  wind.set_turbulence(10.0);
  double spread = 0.0;
  for (int i = 0; i < 2000; ++i) {
    spread = std::max(spread, std::abs(wind.step(0.002 + i * 0.001, 0.001, 5.0, 4.0).z()));
  }
  EXPECT_GT(spread, 0.0);
}
