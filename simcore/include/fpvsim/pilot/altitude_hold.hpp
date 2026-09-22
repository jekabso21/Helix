#pragma once

#include <array>
#include <cstdint>

namespace fpvsim::pilot {

using RcChannels = std::array<std::uint16_t, 16>;

struct AltitudeHoldParams {
  double target_height_m;
  double climb_rate_mps;
  double kp_us_per_m;
  double ki_us_per_m_s;
  double kd_us_per_mps;
  double hover_throttle_us;
  double integral_limit_us;
  double arm_delay_s;
};

// Test pilot: arms after a delay, then holds height with a throttle PID; sticks stay centred
class AltitudeHoldPilot {
 public:
  explicit AltitudeHoldPilot(const AltitudeHoldParams& params);

  RcChannels channels(double sim_time_s, double height_m, double climb_rate_mps, bool armed,
                      double dt_s);
  [[nodiscard]] double target_height_m() const { return target_m_; }

 private:
  AltitudeHoldParams params_;
  double target_m_ = 0.0;
  double integral_us_ = 0.0;
};

}  // namespace fpvsim::pilot
