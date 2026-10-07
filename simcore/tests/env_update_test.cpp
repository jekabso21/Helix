#include <gtest/gtest.h>

#include <stdexcept>

#include <nlohmann/json.hpp>

#include <fpvsim/sim/env_update.hpp>

namespace sim = fpvsim::sim;
namespace env = fpvsim::env;
using nlohmann::json;

namespace {

env::Wind still_air() {
  return env::Wind(env::WindParams{
      .mean_speed_mps = 0.0,
      .mean_from_rad = 0.0,
      .profile = {.log_law = false, .roughness_m = 0.1, .reference_height_m = 10.0},
      .turbulence_w20_mps = 0.0,
      .seed = 1});
}

}  // namespace

TEST(EnvUpdateTest, ParsesEachPartOnItsOwn) {
  const sim::EnvUpdate mean =
      sim::env_update_from_json(json::parse(R"({"wind": {"mean_speed_mps": 7, "mean_from_rad": 1.5}})"));
  EXPECT_TRUE(mean.set_mean);
  EXPECT_FALSE(mean.set_turbulence);
  EXPECT_FALSE(mean.add_gust);
  EXPECT_DOUBLE_EQ(mean.mean_speed_mps, 7.0);
  const sim::EnvUpdate gust = sim::env_update_from_json(
      json::parse(R"({"gust": {"duration_s": 2, "amplitude_mps": 5, "from_rad": 0}})"));
  EXPECT_TRUE(gust.add_gust);
  EXPECT_FALSE(gust.set_mean);
  const sim::EnvUpdate turbulence =
      sim::env_update_from_json(json::parse(R"({"wind": {"turbulence_w20_mps": 7.7}})"));
  EXPECT_TRUE(turbulence.set_turbulence);
  EXPECT_FALSE(turbulence.set_mean);
}

TEST(EnvUpdateTest, RejectsWhatCannotBeApplied) {
  const char* const bad[] = {
      R"({})",
      R"({"wind": {}})",
      R"({"wind": {"mean_speed_mps": 5}})",  // speed without direction
      R"({"wind": {"mean_speed_mps": -1, "mean_from_rad": 0}})",
      R"({"wind": {"mean_speed_mps": "5", "mean_from_rad": 0}})",
      R"({"wind": {"turbulence_w20_mps": -3}})",
      R"({"gust": {"duration_s": 0, "amplitude_mps": 5, "from_rad": 0}})",
      R"({"gust": {"amplitude_mps": 5, "from_rad": 0}})",
      R"({"weather": {}})",
  };
  for (const char* text : bad) {
    EXPECT_THROW((void)sim::env_update_from_json(json::parse(text)), std::invalid_argument) << text;
  }
}

TEST(EnvUpdateTest, AppliesToTheWindAndStartsTheGustNow) {
  env::Wind wind = still_air();
  const sim::EnvUpdate update = sim::env_update_from_json(json::parse(
      R"({"wind": {"mean_speed_mps": 4, "mean_from_rad": 0, "turbulence_w20_mps": 0},
          "gust": {"duration_s": 2, "amplitude_mps": 6, "from_rad": 0}})"));
  ASSERT_TRUE(sim::apply_env_update(wind, update, 30.0));
  EXPECT_DOUBLE_EQ(wind.params().mean_speed_mps, 4.0);
  // at the gust's midpoint the northerly gust adds its full amplitude to the northerly mean
  EXPECT_NEAR(wind.step(31.0, 0.001, 5.0, 4.0).x(), -10.0, 1e-9);
}
