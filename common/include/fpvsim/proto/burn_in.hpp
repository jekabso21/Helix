#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

// Frame counter burned into the image so a consumer of a video output can name the frame it sees
namespace fpvsim::proto {

inline constexpr std::uint32_t kBurnInCellPx = 16;
inline constexpr std::uint32_t kBurnInBits = 16;
inline constexpr std::uint32_t kBurnInCells = kBurnInBits + 2;
inline constexpr std::uint64_t kBurnInModulus = 1ULL << kBurnInBits;

// Cell 0 is white and cell 1 is black, so a blank or noisy image does not read as a counter;
// the remaining cells carry the frame index, most significant bit first.
constexpr bool burn_in_cell_is_white(std::uint64_t frame_index, std::uint32_t cell) {
  if (cell == 0) {
    return true;
  }
  if (cell == 1) {
    return false;
  }
  const std::uint32_t bit = kBurnInBits - 1 - (cell - 2);
  return ((frame_index % kBurnInModulus) >> bit & 1U) != 0;
}

constexpr bool burn_in_fits(std::uint32_t width, std::uint32_t height) {
  return width >= kBurnInCells * kBurnInCellPx && height >= kBurnInCellPx;
}

// Paints the counter over the top left of the image; does nothing if it would not fit
inline void stamp_burn_in(std::span<std::byte> pixels, std::uint32_t width, std::uint32_t height,
                         std::uint32_t stride_bytes, std::uint32_t bytes_per_pixel,
                         std::uint64_t frame_index) {
  if (!burn_in_fits(width, height) ||
      pixels.size() < static_cast<std::size_t>(stride_bytes) * kBurnInCellPx) {
    return;
  }
  for (std::uint32_t cell = 0; cell < kBurnInCells; ++cell) {
    const std::byte value{burn_in_cell_is_white(frame_index, cell) ? std::byte{0xFF}
                                                                  : std::byte{0x00}};
    for (std::uint32_t row = 0; row < kBurnInCellPx; ++row) {
      const std::size_t start =
          static_cast<std::size_t>(row) * stride_bytes + cell * kBurnInCellPx * bytes_per_pixel;
      const std::size_t count = kBurnInCellPx * bytes_per_pixel;
      for (std::size_t i = 0; i < count; ++i) {
        pixels[start + i] = value;
      }
    }
  }
}

// Reads the counter back from an image; empty when the marker cells are not there
inline std::optional<std::uint32_t> read_burn_in(std::span<const std::byte> pixels,
                                                 std::uint32_t width, std::uint32_t height,
                                                 std::uint32_t stride_bytes,
                                                 std::uint32_t bytes_per_pixel) {
  if (!burn_in_fits(width, height) ||
      pixels.size() < static_cast<std::size_t>(stride_bytes) * kBurnInCellPx) {
    return std::nullopt;
  }
  const std::size_t row = static_cast<std::size_t>(kBurnInCellPx / 2) * stride_bytes;
  std::uint32_t index = 0;
  for (std::uint32_t cell = 0; cell < kBurnInCells; ++cell) {
    const std::size_t at =
        row + (cell * kBurnInCellPx + kBurnInCellPx / 2) * static_cast<std::size_t>(bytes_per_pixel);
    const bool white = static_cast<unsigned char>(pixels[at]) >= 128;
    if (cell == 0 && !white) {
      return std::nullopt;
    }
    if (cell == 1 && white) {
      return std::nullopt;
    }
    if (cell >= 2) {
      index = (index << 1U) | (white ? 1U : 0U);
    }
  }
  return index;
}

}  // namespace fpvsim::proto
