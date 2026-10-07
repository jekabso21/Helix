#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <random>

#include <Eigen/Core>

namespace fpvsim::env {

inline constexpr std::size_t kMaxGusts = 8;

struct WindProfile {
  bool log_law;               // false: the mean wind is the same at every height
  double roughness_m;         // z0
  double reference_height_m;  // where the mean speed is the configured one
};

struct WindParams {
  double mean_speed_mps;
  double mean_from_rad;       // meteorological: where the wind comes from, 0 = north, clockwise
  WindProfile profile;
  double turbulence_w20_mps;  // MIL-F-8785C wind speed at 20 ft that sets the intensity; 0 = none
  std::uint64_t seed;
};

// A 1-cosine gust along a direction, starting at an absolute sim time
struct Gust {
  double start_s;
  double duration_s;
  double amplitude_mps;
  double from_rad;
};

struct DrydenScales {
  double sigma_horizontal_mps;  // sigma_u = sigma_v
  double sigma_vertical_mps;    // sigma_w
  double length_horizontal_m;   // L_u = L_v
  double length_vertical_m;     // L_w
};

// Horizontal wind vector in NED, pointing where the air moves to
Eigen::Vector3d wind_from(double speed_mps, double from_rad);
double profile_factor(const WindProfile& profile, double height_m);
double gust_speed(const Gust& gust, double time_s);
// MIL-F-8785C low-altitude scales; height is clamped to the standard's 10..1000 ft
DrydenScales dryden_low_altitude(double w20_mps, double height_m);

// Mean wind with an optional height profile, Dryden turbulence and discrete gusts. Seeded and
// allocation-free, so it can run in the physics loop.
class Wind {
 public:
  explicit Wind(const WindParams& params);

  void set_mean(double speed_mps, double from_rad);
  void set_turbulence(double w20_mps);
  // false when kMaxGusts gusts are already pending
  bool add_gust(const Gust& gust);
  void reset();

  // Wind at the vehicle, NED. air_speed_mps converts the turbulence's spatial scales into time
  // (frozen turbulence carried past the vehicle).
  Eigen::Vector3d step(double time_s, double dt_s, double height_m, double air_speed_mps);

  [[nodiscard]] const WindParams& params() const { return params_; }
  [[nodiscard]] const Eigen::Vector3d& turbulence_ned() const { return turbulence_ned_; }
  [[nodiscard]] std::size_t pending_gusts() const { return gust_count_; }

 private:
  WindParams params_;
  std::mt19937_64 rng_;
  std::normal_distribution<double> normal_;
  Eigen::Vector3d filter_uvw_;  // turbulence along, across and below the mean wind direction
  Eigen::Vector3d turbulence_ned_;
  std::array<Gust, kMaxGusts> gusts_{};
  std::size_t gust_count_ = 0;
};

}  // namespace fpvsim::env
