#include <fpvsim/env/wind.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>

namespace fpvsim::env {

namespace {

constexpr double kFeetPerMetre = 1.0 / 0.3048;
// MIL-F-8785C gives the low-altitude model between 10 and 1000 ft
constexpr double kMinHeightFt = 10.0;
constexpr double kMaxHeightFt = 1000.0;
// Frozen turbulence needs relative motion; a hovering drone in still air still sees slow change
constexpr double kMinAirSpeedMps = 1.0;
// Rotates the session seed so the wind does not share a stream with the sensor noise
constexpr std::uint64_t kSeedSalt = 0x9e3779b97f4a7c15ULL;

}  // namespace

Eigen::Vector3d wind_from(double speed_mps, double from_rad) {
  return {-speed_mps * std::cos(from_rad), -speed_mps * std::sin(from_rad), 0.0};
}

double profile_factor(const WindProfile& profile, double height_m) {
  if (!profile.log_law) {
    return 1.0;
  }
  if (height_m < profile.roughness_m) {
    return 0.0;
  }
  return std::log(height_m / profile.roughness_m) /
         std::log(profile.reference_height_m / profile.roughness_m);
}

double gust_speed(const Gust& gust, double time_s) {
  const double t = time_s - gust.start_s;
  if (t < 0.0 || t > gust.duration_s || gust.duration_s <= 0.0) {
    return 0.0;
  }
  return 0.5 * gust.amplitude_mps * (1.0 - std::cos(2.0 * std::numbers::pi * t / gust.duration_s));
}

DrydenScales dryden_low_altitude(double w20_mps, double height_m) {
  // the standard's formulas are in feet; lengths are converted back to metres
  const double h_ft = std::clamp(height_m * kFeetPerMetre, kMinHeightFt, kMaxHeightFt);
  const double base = 0.177 + 0.000823 * h_ft;
  const double sigma_w = 0.1 * w20_mps;
  return DrydenScales{.sigma_horizontal_mps = sigma_w / std::pow(base, 0.4),
                      .sigma_vertical_mps = sigma_w,
                      .length_horizontal_m = h_ft / std::pow(base, 1.2) / kFeetPerMetre,
                      .length_vertical_m = h_ft / kFeetPerMetre};
}

Wind::Wind(const WindParams& params)
    : params_(params),
      rng_(params.seed ^ kSeedSalt),
      normal_(0.0, 1.0),
      filter_uvw_(Eigen::Vector3d::Zero()),
      turbulence_ned_(Eigen::Vector3d::Zero()) {}

void Wind::set_mean(double speed_mps, double from_rad) {
  params_.mean_speed_mps = speed_mps;
  params_.mean_from_rad = from_rad;
}

void Wind::set_turbulence(double w20_mps) {
  params_.turbulence_w20_mps = w20_mps;
  if (w20_mps <= 0.0) {
    filter_uvw_.setZero();
  }
}

bool Wind::add_gust(const Gust& gust) {
  if (gust_count_ >= kMaxGusts) {
    return false;
  }
  gusts_[gust_count_++] = gust;
  return true;
}

void Wind::reset() {
  filter_uvw_.setZero();
  turbulence_ned_.setZero();
  gust_count_ = 0;
}

Eigen::Vector3d Wind::step(double time_s, double dt_s, double height_m, double air_speed_mps) {
  // the turbulence axes follow the mean wind: u along it, v to its right, w down
  const double toward = params_.mean_from_rad + std::numbers::pi;
  const Eigen::Vector3d u_axis(std::cos(toward), std::sin(toward), 0.0);
  const Eigen::Vector3d v_axis(-u_axis.y(), u_axis.x(), 0.0);

  turbulence_ned_.setZero();
  if (params_.turbulence_w20_mps > 0.0) {
    const DrydenScales s = dryden_low_altitude(params_.turbulence_w20_mps, height_m);
    const double speed = std::max(air_speed_mps, kMinAirSpeedMps);
    // Each component is a first-order Gauss-Markov process with Dryden's sigma and length scale,
    // discretized exactly: x <- a x + sigma sqrt(1 - a^2) n, a = exp(-V dt / L). Variance sigma^2
    // and correlation time L / V hold for any step; Dryden's v and w are second order, which this
    // first-order form approximates.
    const double a_h = std::exp(-speed * dt_s / s.length_horizontal_m);
    const double a_w = std::exp(-speed * dt_s / s.length_vertical_m);
    filter_uvw_.x() = a_h * filter_uvw_.x() +
                      s.sigma_horizontal_mps * std::sqrt(1.0 - a_h * a_h) * normal_(rng_);
    filter_uvw_.y() = a_h * filter_uvw_.y() +
                      s.sigma_horizontal_mps * std::sqrt(1.0 - a_h * a_h) * normal_(rng_);
    filter_uvw_.z() = a_w * filter_uvw_.z() +
                      s.sigma_vertical_mps * std::sqrt(1.0 - a_w * a_w) * normal_(rng_);
    turbulence_ned_ = filter_uvw_.x() * u_axis + filter_uvw_.y() * v_axis +
                      Eigen::Vector3d(0.0, 0.0, filter_uvw_.z());
  }

  Eigen::Vector3d gusts = Eigen::Vector3d::Zero();
  std::size_t kept = 0;
  for (std::size_t i = 0; i < gust_count_; ++i) {
    const Gust& gust = gusts_[i];
    gusts += wind_from(gust_speed(gust, time_s), gust.from_rad);
    if (time_s <= gust.start_s + gust.duration_s) {
      gusts_[kept++] = gust;
    }
  }
  gust_count_ = kept;

  const double mean = params_.mean_speed_mps * profile_factor(params_.profile, height_m);
  return wind_from(mean, params_.mean_from_rad) + turbulence_ned_ + gusts;
}

}  // namespace fpvsim::env
