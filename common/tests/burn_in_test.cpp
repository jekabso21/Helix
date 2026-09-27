#include <fpvsim/proto/burn_in.hpp>

#include <gtest/gtest.h>

#include <cstddef>
#include <vector>

namespace {

using fpvsim::proto::kBurnInCellPx;
using fpvsim::proto::kBurnInCells;
using fpvsim::proto::kBurnInModulus;
using fpvsim::proto::read_burn_in;
using fpvsim::proto::stamp_burn_in;

constexpr std::uint32_t kWidth = 1280;
constexpr std::uint32_t kHeight = 720;
constexpr std::uint32_t kBpp = 3;
constexpr std::uint32_t kStride = kWidth * kBpp;

std::vector<std::byte> grey_frame() {
  return std::vector<std::byte>(static_cast<std::size_t>(kStride) * kHeight, std::byte{0x40});
}

TEST(BurnInTest, RoundTripsEveryIndexOfInterest) {
  const std::uint64_t indices[] = {0, 1, 2, 12345, kBurnInModulus - 1};
  for (std::uint64_t index : indices) {
    auto frame = grey_frame();
    stamp_burn_in(frame, kWidth, kHeight, kStride, kBpp, index);
    EXPECT_EQ(read_burn_in(frame, kWidth, kHeight, kStride, kBpp), index) << "index " << index;
  }
}

TEST(BurnInTest, CounterWrapsAtTheModulus) {
  auto frame = grey_frame();
  stamp_burn_in(frame, kWidth, kHeight, kStride, kBpp, kBurnInModulus + 7);
  EXPECT_EQ(read_burn_in(frame, kWidth, kHeight, kStride, kBpp), 7U);
}

TEST(BurnInTest, LeavesTheRestOfTheImageAlone) {
  auto frame = grey_frame();
  stamp_burn_in(frame, kWidth, kHeight, kStride, kBpp, 42);
  EXPECT_EQ(frame[static_cast<std::size_t>(kStride) * kBurnInCellPx], std::byte{0x40});
  EXPECT_EQ(frame[kBurnInCells * kBurnInCellPx * kBpp], std::byte{0x40});
}

TEST(BurnInTest, AFrameWithoutTheMarkerReadsAsNothing) {
  const auto frame = grey_frame();
  EXPECT_FALSE(read_burn_in(frame, kWidth, kHeight, kStride, kBpp).has_value());
}

TEST(BurnInTest, DoesNothingWhenTheCounterWouldNotFit) {
  constexpr std::uint32_t narrow = kBurnInCells * kBurnInCellPx - 2;
  std::vector<std::byte> frame(static_cast<std::size_t>(narrow) * kBpp * kBurnInCellPx,
                               std::byte{0x40});
  stamp_burn_in(frame, narrow, kBurnInCellPx, narrow * kBpp, kBpp, 3);
  EXPECT_EQ(frame[0], std::byte{0x40});
  EXPECT_FALSE(read_burn_in(frame, narrow, kBurnInCellPx, narrow * kBpp, kBpp).has_value());
}

}  // namespace
