#pragma once

#include <array>
#include <cstddef>

namespace fpvsim::physics {

inline constexpr std::size_t kOcvPoints = 11;  // per-cell open-circuit voltage at SOC 0, 0.1 .. 1

struct BatteryParams {
  int cells;
  double capacity_ah;
  double cell_resistance_ohm;       // at the configured temperature (factor applied upstream)
  double connector_resistance_ohm;  // whole pack
  double avionics_current_a;
  double esc_cutoff_v;
  double initial_soc;
  double rc_resistance_ohm;  // per cell transient branch; 0 disables
  double rc_capacitance_f;
  std::array<double, kOcvPoints> ocv_v;
};

struct BatteryState {
  double soc;
  double v_rc;           // transient branch voltage per cell
  double current_a;      // last step's pack current
  double bus_voltage_v;  // as seen by the ESCs
  double consumed_ah;
  bool cutoff;  // bus voltage fell below esc_cutoff_v
};

double open_circuit_voltage(const BatteryParams& params, double soc);  // per cell

BatteryState initial_battery(const BatteryParams& params);

// Pack current is the motor bus currents plus avionics; the voltage uses the previous current
BatteryState step_battery(const BatteryParams& params, const BatteryState& state,
                          double motor_current_a, double dt_s);

}  // namespace fpvsim::physics
