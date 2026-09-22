#include <fpvsim/pilot/altitude_hold.hpp>

#include <algorithm>

namespace fpvsim::pilot {

namespace {
constexpr std::uint16_t kLow = 1000;
constexpr std::uint16_t kCentre = 1500;
constexpr std::uint16_t kHigh = 2000;
constexpr std::size_t kThrottle = 2;
constexpr std::size_t kArm = 4;
}  // namespace

AltitudeHoldPilot::AltitudeHoldPilot(const AltitudeHoldParams& params) : params_(params) {}

void AltitudeHoldPilot::reset(double sim_time_s) {
  start_s_ = sim_time_s;
  target_m_ = 0.0;
  integral_us_ = 0.0;
}

RcChannels AltitudeHoldPilot::channels(double sim_time_s, double height_m, double climb_rate_mps,
                                       bool armed, double dt_s) {
  RcChannels channels{};
  channels.fill(kLow);
  channels[0] = kCentre;
  channels[1] = kCentre;
  channels[3] = kCentre;
  channels[kArm] = sim_time_s - start_s_ >= params_.arm_delay_s ? kHigh : kLow;
  if (!armed) {
    channels[kThrottle] = kLow;
    return channels;
  }
  target_m_ = std::min(params_.target_height_m, target_m_ + params_.climb_rate_mps * dt_s);
  const double error = target_m_ - height_m;
  integral_us_ = std::clamp(integral_us_ + params_.ki_us_per_m_s * error * dt_s,
                            -params_.integral_limit_us, params_.integral_limit_us);
  const double throttle = params_.hover_throttle_us + params_.kp_us_per_m * error + integral_us_ -
                          params_.kd_us_per_mps * climb_rate_mps;
  channels[kThrottle] = static_cast<std::uint16_t>(std::clamp(throttle, 1000.0, 2000.0));
  return channels;
}

}  // namespace fpvsim::pilot
