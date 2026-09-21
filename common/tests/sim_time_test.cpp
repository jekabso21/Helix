#include <doctest/doctest.h>

#include <fpvsim/sim_time.hpp>

using fpvsim::SimTime;

TEST_CASE("step_for_1000_hz_is_one_millisecond") {
  CHECK(fpvsim::step_from_rate_hz(1000) == SimTime{1'000'000});
}

TEST_CASE("rate_that_does_not_divide_one_second_is_rejected") {
  CHECK_FALSE(fpvsim::step_from_rate_hz(3).has_value());
  CHECK_FALSE(fpvsim::step_from_rate_hz(0).has_value());
  CHECK_FALSE(fpvsim::step_from_rate_hz(-1000).has_value());
}

TEST_CASE("one_hour_of_steps_accumulates_without_drift") {
  const SimTime step = fpvsim::step_from_rate_hz(8000).value_or(SimTime{});
  REQUIRE(step.ns == 125'000);
  SimTime t{};
  const std::int64_t steps = 8000LL * 3600;
  for (std::int64_t i = 0; i < steps; ++i) {
    t += step;
  }
  CHECK(t.ns == 3600 * fpvsim::kNanosecondsPerSecond);
  CHECK(t == step * steps);
}

TEST_CASE("to_seconds_converts_nanoseconds") {
  CHECK(fpvsim::to_seconds(SimTime{1'500'000'000}) == doctest::Approx(1.5).epsilon(1e-15));
}

TEST_CASE("sim_times_compare_by_value") {
  CHECK(SimTime{1} < SimTime{2});
  CHECK(SimTime{5} - SimTime{2} == SimTime{3});
}
