#include <gtest/gtest.h>

#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <numbers>

#include <fpvsim/constants.hpp>
#include <fpvsim/physics/motor.hpp>

#include "test_quad.hpp"

namespace physics = fpvsim::physics;
namespace quad = fpvsim::testing;

namespace {

struct TableRow {
  double throttle;
  double voltage_v;
  double rpm;
  double current_a;
  double thrust_n;
  double air_density_kg_m3;
};

// The same columns simtools.propfit reads; thrust_g is converted to newtons
std::vector<TableRow> read_table(const std::string& path) {
  std::ifstream file(path);
  EXPECT_TRUE(file.is_open()) << path;
  std::string line;
  std::getline(file, line);
  std::vector<std::string> header;
  for (std::stringstream columns(line); std::getline(columns, line, ',');) {
    header.push_back(line);
  }
  std::vector<TableRow> rows;
  while (std::getline(file, line)) {
    if (line.empty()) {
      continue;
    }
    std::vector<double> values;
    for (std::stringstream columns(line); std::getline(columns, line, ',');) {
      values.push_back(std::stod(line));
    }
    TableRow row{};
    row.air_density_kg_m3 = 1.225;
    for (std::size_t i = 0; i < header.size() && i < values.size(); ++i) {
      if (header[i] == "throttle") {
        row.throttle = values[i];
      } else if (header[i] == "voltage_v") {
        row.voltage_v = values[i];
      } else if (header[i] == "rpm") {
        row.rpm = values[i];
      } else if (header[i] == "current_a") {
        row.current_a = values[i];
      } else if (header[i] == "thrust_g") {
        row.thrust_n = values[i] * 1e-3 * fpvsim::kStandardGravityMps2;
      } else if (header[i] == "thrust_n") {
        row.thrust_n = values[i];
      } else if (header[i] == "air_density_kg_m3") {
        row.air_density_kg_m3 = values[i];
      }
    }
    rows.push_back(row);
  }
  return rows;
}

physics::MotorOutput settle(const physics::MotorParams& motor, const physics::PropParams& prop,
                            const TableRow& row) {
  physics::MotorInput input = quad::still_air(row.throttle);
  input.bus_voltage_v = row.voltage_v;
  input.air_density_kg_m3 = row.air_density_kg_m3;
  physics::MotorOutput output{};
  double speed = 0.0;
  for (int i = 0; i < 5000; ++i) {
    output = physics::step_motor(motor, prop, speed, input, 0.001);
    speed = output.speed_radps;
  }
  return output;
}

}  // namespace

// Acceptance: the C++ model reproduces the table the Python fit was derived from, within 5 %
TEST(ThrustTableTest, SteadyStateMatchesTheTableWithinFivePercent) {
  const std::string path =
      std::string(FPVSIM_DATA_DIR) + "/reference/generic_2207_1900kv_5x4.3x3_6s_synthetic.csv";
  const std::vector<TableRow> rows = read_table(path);
  ASSERT_GE(rows.size(), 10U);
  const physics::MotorParams motor = quad::dc_motor();
  const physics::PropParams prop = quad::quad_prop();
  for (const TableRow& row : rows) {
    if (row.rpm < 1000.0) {
      continue;  // below the idle point the table is dominated by the no-load current
    }
    const physics::MotorOutput output = settle(motor, prop, row);
    const double rpm = output.speed_radps * 60.0 / (2.0 * std::numbers::pi);
    EXPECT_NEAR(rpm, row.rpm, 0.05 * row.rpm) << "throttle " << row.throttle;
    EXPECT_NEAR(output.thrust_n, row.thrust_n, 0.05 * row.thrust_n) << "throttle " << row.throttle;
    EXPECT_NEAR(output.current_a, row.current_a, 0.05 * row.current_a)
        << "throttle " << row.throttle;
  }
}
