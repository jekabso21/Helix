#include <fpvsim/physics/motor.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>

namespace fpvsim::physics {

namespace {

constexpr double kMinVoltage = 0.5;  // below this the first-order current estimate is meaningless

double density_ratio(const PropParams& prop, double air_density_kg_m3) {
  return air_density_kg_m3 / prop.reference_density_kg_m3;
}

}  // namespace

double quantize_command(double command) {
  return std::round(std::clamp(command, 0.0, 1.0) * kDshotSteps) / kDshotSteps;
}

double inflow_efficiency(const PropParams& prop, double speed_radps, double axial_inflow_mps) {
  if (prop.inflow_coefficient <= 0.0 || axial_inflow_mps <= 0.0 || speed_radps <= 0.0) {
    return 1.0;
  }
  const double pitch_speed = prop.pitch_m * speed_radps / (2.0 * std::numbers::pi);
  return std::clamp(1.0 - prop.inflow_coefficient * axial_inflow_mps / pitch_speed, 0.0, 1.0);
}

double ground_effect_factor(const PropParams& prop, double height_above_ground_m) {
  const double z = std::max(height_above_ground_m, prop.radius_m / 2.0);
  const double ratio = prop.radius_m / (4.0 * z);
  return 1.0 / (1.0 - ratio * ratio);
}

MotorOutput step_motor(const MotorParams& motor, const PropParams& prop, double speed_radps,
                       const MotorInput& input, double dt_s) {
  const double command = quantize_command(input.command);
  const double rho = density_ratio(prop, input.air_density_kg_m3);
  const double drag_torque_before = prop.torque_coefficient * rho * speed_radps * speed_radps;
  double next = 0.0;
  double current = 0.0;
  double bus_current = 0.0;
  if (motor.model == MotorModel::kDc) {
    // J w' = Kt (I - I0) - Q, I = (u V - Ke w) / R; back-EMF term taken implicitly
    const double kt = 1.0 / motor.kv_radps_per_v;  // Ke = Kt in SI
    const double v_m = command * input.bus_voltage_v;
    const double no_load = speed_radps > 0.0 ? motor.no_load_current_a : 0.0;
    const double gain = dt_s * kt * kt / (motor.rotor_inertia_kg_m2 * motor.resistance_ohm);
    const double drive = dt_s / motor.rotor_inertia_kg_m2 *
                         (kt * v_m / motor.resistance_ohm - kt * no_load - drag_torque_before);
    next = std::max(0.0, (speed_radps + drive) / (1.0 + gain));
    current = std::max((v_m - kt * next) / motor.resistance_ohm, -motor.brake_current_a);
    bus_current = std::max(command * current, 0.0);
  } else {
    const double voltage_ratio = std::max(input.bus_voltage_v, 0.0) / motor.reference_voltage_v;
    const double target = command * motor.max_speed_radps * voltage_ratio;
    const double blend = 1.0 - std::exp(-dt_s / motor.time_constant_s);
    next = std::max(0.0, speed_radps + (target - speed_radps) * blend);
    // no electrical model: current from shaft power at the applied voltage plus the no-load draw
    const double v_m = command * input.bus_voltage_v;
    if (v_m > kMinVoltage && next > 0.0) {
      const double shaft_power = prop.torque_coefficient * rho * next * next * next;
      current = shaft_power / v_m + motor.no_load_current_a;
      bus_current = command * current;
    }
  }
  const double acceleration = (next - speed_radps) / dt_s;
  const double drag_torque = prop.torque_coefficient * rho * next * next;
  const double efficiency = inflow_efficiency(prop, next, input.axial_inflow_mps) *
                            ground_effect_factor(prop, input.height_above_ground_m);
  return MotorOutput{.speed_radps = next,
                     .thrust_n = prop.thrust_coefficient * rho * next * next * efficiency,
                     .reaction_torque_nm = drag_torque + motor.rotor_inertia_kg_m2 * acceleration,
                     .current_a = current,
                     .bus_current_a = bus_current};
}

double hover_command(const MotorParams& motor, const PropParams& prop, double thrust_n,
                     double bus_voltage_v) {
  const double speed = std::sqrt(thrust_n / prop.thrust_coefficient);
  if (motor.model == MotorModel::kDc) {
    // steady state: Kt (I - I0) = k_q w^2 and u V = I R + Ke w
    const double kt = 1.0 / motor.kv_radps_per_v;
    const double current = motor.no_load_current_a + prop.torque_coefficient * speed * speed / kt;
    return std::clamp((current * motor.resistance_ohm + kt * speed) / bus_voltage_v, 0.0, 1.0);
  }
  return std::clamp(speed / motor.max_speed_radps * motor.reference_voltage_v / bus_voltage_v, 0.0,
                    1.0);
}

}  // namespace fpvsim::physics
