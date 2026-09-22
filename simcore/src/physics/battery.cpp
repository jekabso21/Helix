#include <fpvsim/physics/battery.hpp>

#include <algorithm>
#include <cmath>

namespace fpvsim::physics {

double open_circuit_voltage(const BatteryParams& params, double soc) {
  const double x = std::clamp(soc, 0.0, 1.0) * static_cast<double>(kOcvPoints - 1);
  const auto lower = static_cast<std::size_t>(std::floor(x));
  const std::size_t upper = std::min(lower + 1, kOcvPoints - 1);
  const double t = x - static_cast<double>(lower);
  return params.ocv_v[lower] + (params.ocv_v[upper] - params.ocv_v[lower]) * t;
}

BatteryState initial_battery(const BatteryParams& params) {
  const double cells = static_cast<double>(params.cells);
  return BatteryState{.soc = params.initial_soc,
                      .v_rc = 0.0,
                      .current_a = 0.0,
                      .bus_voltage_v = cells * open_circuit_voltage(params, params.initial_soc),
                      .consumed_ah = 0.0,
                      .cutoff = false};
}

BatteryState step_battery(const BatteryParams& params, const BatteryState& state,
                          double motor_current_a, double dt_s) {
  const double current = std::max(motor_current_a, 0.0) + params.avionics_current_a;
  const double cells = static_cast<double>(params.cells);
  const double soc =
      std::clamp(state.soc - current * dt_s / (3600.0 * params.capacity_ah), 0.0, 1.0);
  double v_rc = 0.0;
  if (params.rc_resistance_ohm > 0.0 && params.rc_capacitance_f > 0.0) {
    const double tau = params.rc_resistance_ohm * params.rc_capacitance_f;
    const double target = current * params.rc_resistance_ohm;
    v_rc = target + (state.v_rc - target) * std::exp(-dt_s / tau);
  }
  const double cell_v =
      open_circuit_voltage(params, soc) - current * params.cell_resistance_ohm - v_rc;
  const double bus = std::max(cells * cell_v - current * params.connector_resistance_ohm, 0.0);
  return BatteryState{.soc = soc,
                      .v_rc = v_rc,
                      .current_a = current,
                      .bus_voltage_v = bus,
                      .consumed_ah = state.consumed_ah + current * dt_s / 3600.0,
                      .cutoff = state.cutoff || bus < params.esc_cutoff_v};
}

}  // namespace fpvsim::physics
