#include <gtest/gtest.h>

#include <fpvsim/env/atmosphere.hpp>

namespace env = fpvsim::env;

namespace {
env::AtmosphereParams ground(double temperature_k) {
  return env::AtmosphereParams{.ground_temperature_k = temperature_k,
                               .ground_pressure_pa = 101325.0};
}
}  // namespace

TEST(AtmosphereTest, StandardSeaLevelDensityIs1225) {
  const env::Air air = env::air_at_height(ground(288.15), 0.0);
  EXPECT_NEAR(air.density_kg_m3, 1.225, 1e-3);
  EXPECT_DOUBLE_EQ(air.pressure_pa, 101325.0);
}

TEST(AtmosphereTest, PressureAt1000MetresMatchesStandardAtmosphere) {
  const env::Air air = env::air_at_height(ground(288.15), 1000.0);
  EXPECT_NEAR(air.pressure_pa, 89874.6, 5.0);
  EXPECT_NEAR(air.temperature_k, 281.65, 1e-9);
}

TEST(AtmosphereTest, GroundDensityRatioBetweenColdAndHotIsInverseTemperatureRatio) {
  const env::Air cold = env::air_at_height(ground(253.15), 0.0);
  const env::Air hot = env::air_at_height(ground(303.15), 0.0);
  EXPECT_NEAR(cold.density_kg_m3 / hot.density_kg_m3, 303.15 / 253.15, 1e-12);
}
