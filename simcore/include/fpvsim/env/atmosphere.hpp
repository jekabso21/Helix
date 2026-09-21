#pragma once

namespace fpvsim::env {

struct AtmosphereParams {
  double ground_temperature_k;
  double ground_pressure_pa;
};

struct Air {
  double temperature_k;
  double pressure_pa;
  double density_kg_m3;
};

// International Standard Atmosphere anchored at the origin; height is metres above the origin
Air air_at_height(const AtmosphereParams& params, double height_m);

}  // namespace fpvsim::env
