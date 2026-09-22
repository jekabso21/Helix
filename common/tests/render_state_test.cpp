#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include <fpvsim/proto/render_state.hpp>

namespace proto = fpvsim::proto;

namespace {

// Keep in sync with tools/tests/test_render_state.py
proto::RenderState sample() {
  return proto::RenderState{.sim_time_ns = 1'234'567'890,
                            .position_ned = {1.5, -2.25, -10.0},
                            .q_ned_from_frd = {0.8, 0.1, -0.2, 0.3},
                            .velocity_ned = {3.0, 4.0, -0.5},
                            .angular_rate_frd = {0.1, -0.2, 0.3},
                            .armed = 1,
                            .crashed = 0,
                            .motor_count = 4,
                            .motor_rpm = {20000.0F, 21000.0F, 22000.0F, 23000.0F, 0, 0, 0, 0},
                            .sun_dir_ned = {0.5F, 0.5F, -0.75F},
                            .sun_intensity = 0.875F,
                            .fog_density = 0.0F,
                            .precip_type = 2,
                            .precip_intensity = 0.25F,
                            .wind_ned = {5.0F, -1.0F, 0.0F},
                            .video_fault_flags = 5};
}

std::vector<std::byte> read_golden() {
  std::ifstream file(std::string(FPVSIM_GOLDEN_DIR) + "/proto/render_state_v1.bin",
                     std::ios::binary);
  std::vector<char> chars((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  std::vector<std::byte> bytes(chars.size());
  for (std::size_t i = 0; i < chars.size(); ++i) {
    bytes[i] = static_cast<std::byte>(chars[i]);
  }
  return bytes;
}

}  // namespace

TEST(RenderStateTest, SampleMatchesGoldenBytes) {
  std::array<std::byte, proto::kRenderStateMessageSize> buffer{};
  const std::size_t size = proto::serialize_render_state(sample(), 7, buffer);
  ASSERT_EQ(size, proto::kRenderStateMessageSize);
  const std::vector<std::byte> golden = read_golden();
  ASSERT_EQ(golden.size(), buffer.size());
  for (std::size_t i = 0; i < golden.size(); ++i) {
    ASSERT_EQ(golden[i], buffer[i]) << "byte " << i;
  }
}

TEST(RenderStateTest, RoundTripsAndRejectsBadInput) {
  std::array<std::byte, proto::kRenderStateMessageSize> buffer{};
  proto::serialize_render_state(sample(), 7, buffer);
  const auto parsed = proto::parse_render_state(buffer);
  ASSERT_TRUE(parsed.has_value());
  const proto::ParsedRenderState p = parsed.value_or(proto::ParsedRenderState{});
  EXPECT_EQ(p.seq, 7U);
  EXPECT_EQ(p.state.sim_time_ns, 1'234'567'890);
  EXPECT_DOUBLE_EQ(p.state.q_ned_from_frd[2], -0.2);
  EXPECT_EQ(p.state.motor_count, 4);
  EXPECT_FLOAT_EQ(p.state.motor_rpm[3], 23000.0F);
  EXPECT_EQ(p.state.precip_type, 2);
  EXPECT_EQ(p.state.video_fault_flags, 5U);

  EXPECT_FALSE(proto::parse_render_state(std::span(buffer).first(100)).has_value());
  std::array<std::byte, proto::kRenderStateMessageSize> zeros{};
  EXPECT_FALSE(proto::parse_render_state(zeros).has_value());
  std::array<std::byte, 10> small{};
  EXPECT_EQ(proto::serialize_render_state(sample(), 1, small), 0U);
}
