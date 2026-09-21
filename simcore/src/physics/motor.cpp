#include <fpvsim/physics/motor.hpp>

#include <algorithm>
#include <cmath>

namespace fpvsim::physics {

MotorOutput step_motor(const MotorParams& params, double speed_radps, double command, double dt_s) {
  const double target = std::clamp(command, 0.0, 1.0) * params.max_speed_radps;
  const double blend = 1.0 - std::exp(-dt_s / params.time_constant_s);
  const double next = std::max(0.0, speed_radps + (target - speed_radps) * blend);
  const double acceleration = (next - speed_radps) / dt_s;
  const double drag_torque = params.torque_coefficient * next * next;
  return MotorOutput{.speed_radps = next,
                     .thrust_n = params.thrust_coefficient * next * next,
                     .reaction_torque_nm = drag_torque + params.rotor_inertia_kg_m2 * acceleration};
}

double hover_command(const MotorParams& params, double thrust_n) {
  return std::sqrt(thrust_n / params.thrust_coefficient) / params.max_speed_radps;
}

}  // namespace fpvsim::physics
