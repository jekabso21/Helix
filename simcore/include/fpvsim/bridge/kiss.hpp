#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

namespace fpvsim::bridge::kiss {

inline constexpr std::size_t kFrameSize = 10;
inline constexpr double kErpmPerLsb = 100.0;

struct EscTelemetry {
  double temperature_c;
  double voltage_v;
  double current_a;
  double consumption_mah;
  double erpm;
};

// CRC8, polynomial 0x07, initial 0, as in Betaflight's esc_sensor.c
std::uint8_t crc8(std::span<const std::byte> data);

// 10 bytes big-endian: temperature C, 0.01 V, 0.01 A, mAh, eRPM / 100, CRC8; values clamped
std::array<std::byte, kFrameSize> encode_frame(const EscTelemetry& telemetry);

}  // namespace fpvsim::bridge::kiss
