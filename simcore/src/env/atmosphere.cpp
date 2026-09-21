#include <fpvsim/env/atmosphere.hpp>

#include <cmath>

#include <fpvsim/constants.hpp>

namespace fpvsim::env {

Air air_at_height(const AtmosphereParams& params, double height_m) {
  const double temperature = params.ground_temperature_k - kIsaLapseRateKPerM * height_m;
  const double pressure = params.ground_pressure_pa *
                          std::pow(temperature / params.ground_temperature_k, kIsaPressureExponent);
  return Air{.temperature_k = temperature,
             .pressure_pa = pressure,
             .density_kg_m3 = pressure / (kAirGasConstantJPerKgK * temperature)};
}

}  // namespace fpvsim::env
