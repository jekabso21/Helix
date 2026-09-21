#pragma once

namespace fpvsim::physics {

struct MotorParams {
  double max_speed_radps;
  double time_constant_s;
  double thrust_coefficient;  // N per (rad/s)^2
  double torque_coefficient;  // N m per (rad/s)^2
  double rotor_inertia_kg_m2;
};

struct MotorOutput {
  double speed_radps;
  double thrust_n;
  double reaction_torque_nm;  // on the frame, about the thrust axis, before the spin sign
};

// First-order speed response, exact for a command held over the step
MotorOutput step_motor(const MotorParams& params, double speed_radps, double command, double dt_s);

double hover_command(const MotorParams& params, double thrust_n);

}  // namespace fpvsim::physics
