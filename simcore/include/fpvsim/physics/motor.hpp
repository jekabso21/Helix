#pragma once

#include <cstdint>

namespace fpvsim::physics {

enum class MotorModel : std::uint8_t { kFirstOrder, kDc };

struct MotorParams {
  MotorModel model;
  double max_speed_radps;      // first order: speed at full command and reference voltage
  double time_constant_s;      // first order
  double reference_voltage_v;  // first order: max speed scales with bus voltage / reference
  double kv_radps_per_v;       // dc
  double resistance_ohm;       // dc
  double no_load_current_a;    // dc, taken while spinning
  double brake_current_a;      // dc: largest regenerative current the ESC allows
  double rotor_inertia_kg_m2;
  int pole_pairs;
};

struct PropParams {
  double thrust_coefficient;  // N per (rad/s)^2 at reference density
  double torque_coefficient;  // N m per (rad/s)^2 at reference density
  double reference_density_kg_m3;
  double radius_m;
  double pitch_m;
  double inflow_coefficient;      // c_in: thrust loss with axial inflow, 0 disables
  double rotor_drag_coefficient;  // k_h: N per (rad/s) per (m/s) of in-plane airspeed
  int blades;
};

struct MotorInput {
  double command;  // 0..1 before quantization
  double bus_voltage_v;
  double air_density_kg_m3;
  double axial_inflow_mps;       // hub airspeed along the thrust axis, positive when climbing
  double height_above_ground_m;  // rotor hub above the ground, for ground effect
};

struct MotorOutput {
  double speed_radps;
  double thrust_n;
  double reaction_torque_nm;  // on the frame, about the thrust axis, before the spin sign
  double current_a;           // motor phase-equivalent current
  double bus_current_a;       // drawn from the battery by this ESC
};

inline constexpr double kDshotSteps = 1999.0;

double quantize_command(double command);

// Thrust factors: inflow loss for forward speed along the axis, ground effect near the ground
double inflow_efficiency(const PropParams& prop, double speed_radps, double axial_inflow_mps);
double ground_effect_factor(const PropParams& prop, double height_above_ground_m);

MotorOutput step_motor(const MotorParams& motor, const PropParams& prop, double speed_radps,
                       const MotorInput& input, double dt_s);

// Command that holds thrust_n in still air far from the ground at bus_voltage_v
double hover_command(const MotorParams& motor, const PropParams& prop, double thrust_n,
                     double bus_voltage_v);

}  // namespace fpvsim::physics
