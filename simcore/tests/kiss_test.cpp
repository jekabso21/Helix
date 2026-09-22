#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstring>

#include <fpvsim/bridge/kiss.hpp>

namespace kiss = fpvsim::bridge::kiss;

namespace {

std::array<std::byte, 9> bytes9(const char* text) {
  std::array<std::byte, 9> out{};
  std::memcpy(out.data(), text, 9);
  return out;
}

}  // namespace

TEST(KissTest, Crc8MatchesKnownValuesForPolynomial07) {
  EXPECT_EQ(kiss::crc8({}), 0x00);
  const std::array<std::byte, 1> one{std::byte{0x01}};
  EXPECT_EQ(kiss::crc8(one), 0x07);
  EXPECT_EQ(kiss::crc8(bytes9("123456789")), 0xF4);
}

// Same case as tools/tests/test_kiss.py so both encoders agree byte for byte
TEST(KissTest, FramePacksFieldsBigEndianWithScaling) {
  const auto frame = kiss::encode_frame({.temperature_c = 41.0,
                                         .voltage_v = 23.5,
                                         .current_a = 7.5,
                                         .consumption_mah = 123.0,
                                         .erpm = 140000.0});
  const std::array<std::uint8_t, 9> expected{0x29, 0x09, 0x2e, 0x02, 0xee, 0x00, 0x7b, 0x05, 0x78};
  for (std::size_t i = 0; i < 9; ++i) {
    EXPECT_EQ(static_cast<std::uint8_t>(frame[i]), expected[i]) << "byte " << i;
  }
  EXPECT_EQ(static_cast<std::uint8_t>(frame[9]), kiss::crc8(std::span(frame).first(9)));
}

TEST(KissTest, FrameClampsOutOfRangeValues) {
  const auto frame = kiss::encode_frame({.temperature_c = -5.0,
                                         .voltage_v = 1000.0,
                                         .current_a = -1.0,
                                         .consumption_mah = 1e9,
                                         .erpm = 1e9});
  const std::array<std::uint8_t, 9> expected{0x00, 0xff, 0xff, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff};
  for (std::size_t i = 0; i < 9; ++i) {
    EXPECT_EQ(static_cast<std::uint8_t>(frame[i]), expected[i]) << "byte " << i;
  }
}
