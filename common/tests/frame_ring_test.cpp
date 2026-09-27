#include <gtest/gtest.h>

#include <atomic>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include <fpvsim/proto/frame_ring.hpp>

namespace proto = fpvsim::proto;

namespace {

proto::RingSpec tiny_spec(proto::PixelFormat format = proto::PixelFormat::kRgb8) {
  return proto::RingSpec{
      .width = 4, .height = 3, .slot_count = 3, .pixel_format = format, .fps_nominal = 60.0};
}

// Unique per test so parallel ctest runs never share a /dev/shm object
std::string unique_name(const std::string& suffix) {
  return "test_" + std::to_string(::getpid()) + "_" + suffix;
}

proto::FrameMeta meta_for(std::uint64_t index) {
  return proto::FrameMeta{.sim_time_ns = static_cast<std::int64_t>(index) * 16'666'667,
                          .frame_index = index,
                          .camera_position_ned = {static_cast<double>(index), 2.0, -3.0},
                          .q_ned_from_camera = {1.0, 0.0, 0.0, 0.0}};
}

std::vector<std::byte> pattern(std::size_t size, std::byte value) {
  return std::vector<std::byte>(size, value);
}

}  // namespace

TEST(FrameRingTest, LayoutMatchesTheDocumentedSizes) {
  const proto::RingSpec spec{.width = 1280,
                             .height = 720,
                             .slot_count = 4,
                             .pixel_format = proto::PixelFormat::kRgba8,
                             .fps_nominal = 60.0};
  EXPECT_EQ(proto::stride_bytes(spec), 1280U * 4U);
  EXPECT_EQ(proto::slot_size_bytes(spec) % proto::kFrameSlotAlignment, 0U);
  EXPECT_GE(proto::slot_size_bytes(spec), 128U + 1280U * 720U * 4U + 8U);
  EXPECT_EQ(proto::ring_size_bytes(spec), 128U + proto::slot_size_bytes(spec) * 4U);
  EXPECT_EQ(proto::bytes_per_pixel(proto::PixelFormat::kRgb8), 3U);
  EXPECT_EQ(proto::ring_path("main_fpv"), "/fpvsim.main_fpv");
}

TEST(FrameRingTest, ReaderSeesTheHeaderTheWriterPublished) {
  const std::string name = unique_name("header");
  const proto::FrameRingWriter writer(name, tiny_spec());
  const proto::FrameRingReader reader(name);
  const proto::RingSpec seen = reader.spec();
  EXPECT_EQ(seen.width, 4U);
  EXPECT_EQ(seen.height, 3U);
  EXPECT_EQ(seen.slot_count, 3U);
  EXPECT_EQ(seen.pixel_format, proto::PixelFormat::kRgb8);
  EXPECT_DOUBLE_EQ(seen.fps_nominal, 60.0);
  EXPECT_EQ(reader.frame_bytes(), 4U * 3U * 3U);
  EXPECT_EQ(reader.latest_seq(), 0U);
}

TEST(FrameRingTest, RoundTripsFramesAndOnlyReportsNewOnes) {
  const std::string name = unique_name("roundtrip");
  proto::FrameRingWriter writer(name, tiny_spec());
  proto::FrameRingReader reader(name);
  std::vector<std::byte> pixels(reader.frame_bytes());
  proto::FrameMeta meta{};

  EXPECT_EQ(reader.read_latest(0, meta, pixels), 0U);  // nothing published yet

  const auto sent = pattern(writer.frame_bytes(), std::byte{0x5A});
  EXPECT_EQ(writer.write(meta_for(7), sent), 1U);
  EXPECT_EQ(reader.read_latest(0, meta, pixels), 1U);
  EXPECT_EQ(meta.frame_index, 7U);
  EXPECT_EQ(meta.sim_time_ns, 7 * 16'666'667);
  EXPECT_DOUBLE_EQ(meta.camera_position_ned[0], 7.0);
  EXPECT_EQ(pixels, sent);

  EXPECT_EQ(reader.read_latest(1, meta, pixels), 0U);  // same frame is not new
  EXPECT_EQ(writer.write(meta_for(8), pattern(writer.frame_bytes(), std::byte{0x11})), 2U);
  EXPECT_EQ(reader.read_latest(1, meta, pixels), 2U);
  EXPECT_EQ(meta.frame_index, 8U);
}

TEST(FrameRingTest, SlotsWrapAndTheNewestFrameIsAlwaysReadable) {
  const std::string name = unique_name("wrap");
  proto::FrameRingWriter writer(name, tiny_spec());
  proto::FrameRingReader reader(name);
  std::vector<std::byte> pixels(reader.frame_bytes());
  proto::FrameMeta meta{};
  std::uint64_t last = 0;
  for (std::uint64_t i = 1; i <= 10; ++i) {  // more than slot_count, so the ring wraps
    writer.write(meta_for(i), pattern(writer.frame_bytes(), static_cast<std::byte>(i)));
    last = reader.read_latest(last, meta, pixels);
    ASSERT_EQ(last, i);
    EXPECT_EQ(meta.frame_index, i);
    EXPECT_EQ(pixels.front(), static_cast<std::byte>(i));
  }
}

TEST(FrameRingTest, WriterRejectsAFrameOfTheWrongSizeAndTooFewSlots) {
  const std::string name = unique_name("reject");
  proto::FrameRingWriter writer(name, tiny_spec());
  EXPECT_THROW(writer.write(meta_for(1), pattern(writer.frame_bytes() - 1, std::byte{0})),
               std::runtime_error);
  proto::RingSpec bad = tiny_spec();
  bad.slot_count = 2;
  EXPECT_THROW(proto::FrameRingWriter(unique_name("bad"), bad), std::runtime_error);
}

TEST(FrameRingTest, OpeningAMissingRingFails) {
  EXPECT_THROW(proto::FrameRingReader(unique_name("absent")), std::runtime_error);
}

// The reader must never hand out a frame the writer was overwriting: every frame it does accept
// is internally consistent, and it never blocks the writer.
TEST(FrameRingTest, ConcurrentWriterNeverYieldsATornFrame) {
  const std::string name = unique_name("torn");
  const proto::RingSpec spec{.width = 320,
                             .height = 240,
                             .slot_count = 3,
                             .pixel_format = proto::PixelFormat::kRgb8,
                             .fps_nominal = 60.0};
  proto::FrameRingWriter writer(name, spec);
  std::atomic<bool> running{true};
  std::atomic<std::uint64_t> written{0};
  std::thread producer([&] {
    std::vector<std::byte> buffer(writer.frame_bytes());
    for (std::uint64_t i = 1; running.load(std::memory_order_relaxed); ++i) {
      // every byte of a frame carries the same value, so a torn frame is visible
      std::fill(buffer.begin(), buffer.end(), static_cast<std::byte>(i & 0xFF));
      writer.write(meta_for(i), buffer);
      written.store(i, std::memory_order_relaxed);
    }
  });

  proto::FrameRingReader reader(name);
  std::vector<std::byte> pixels(reader.frame_bytes());
  proto::FrameMeta meta{};
  std::uint64_t accepted = 0;
  std::uint64_t last = 0;
  while (accepted < 200 && written.load(std::memory_order_relaxed) < 200000) {
    const std::uint64_t seq = reader.read_latest(last, meta, pixels);
    if (seq == 0) {
      continue;
    }
    last = seq;
    ++accepted;
    const auto expected = static_cast<std::byte>(meta.frame_index & 0xFF);
    ASSERT_EQ(pixels.front(), expected) << "seq " << seq;
    ASSERT_EQ(pixels.back(), expected) << "seq " << seq;
    ASSERT_EQ(pixels[pixels.size() / 2], expected) << "seq " << seq;
  }
  running.store(false, std::memory_order_relaxed);
  producer.join();
  EXPECT_GE(accepted, 200U) << "the reader never caught a complete frame";
}
