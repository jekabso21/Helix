#include <fpvsim/bridge/kiss.hpp>

#include <algorithm>
#include <cmath>

namespace fpvsim::bridge::kiss {

namespace {

std::uint16_t clamp16(double value) {
  return static_cast<std::uint16_t>(std::clamp(std::llround(value), 0LL, 0xFFFFLL));
}

std::uint8_t clamp8(double value) {
  return static_cast<std::uint8_t>(std::clamp(std::llround(value), 0LL, 0xFFLL));
}

void put16(std::array<std::byte, kFrameSize>& frame, std::size_t at, std::uint16_t value) {
  frame[at] = static_cast<std::byte>(value >> 8);
  frame[at + 1] = static_cast<std::byte>(value & 0xFF);
}

}  // namespace

std::uint8_t crc8(std::span<const std::byte> data) {
  std::uint8_t crc = 0;
  for (const std::byte b : data) {
    crc ^= static_cast<std::uint8_t>(b);
    for (int i = 0; i < 8; ++i) {
      crc = static_cast<std::uint8_t>((crc & 0x80) != 0 ? (crc << 1) ^ 0x07 : crc << 1);
    }
  }
  return crc;
}

std::array<std::byte, kFrameSize> encode_frame(const EscTelemetry& telemetry) {
  std::array<std::byte, kFrameSize> frame{};
  frame[0] = static_cast<std::byte>(clamp8(telemetry.temperature_c));
  put16(frame, 1, clamp16(telemetry.voltage_v * 100.0));
  put16(frame, 3, clamp16(telemetry.current_a * 100.0));
  put16(frame, 5, clamp16(telemetry.consumption_mah));
  put16(frame, 7, clamp16(telemetry.erpm / kErpmPerLsb));
  frame[9] = static_cast<std::byte>(crc8(std::span(frame).first(9)));
  return frame;
}

}  // namespace fpvsim::bridge::kiss
