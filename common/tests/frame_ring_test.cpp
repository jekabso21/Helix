#include <gtest/gtest.h>

#include <sys/file.h>
#include <sys/wait.h>

#include <atomic>
#include <cstring>
#include <optional>
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

// A publisher that restarts begins at sequence 1 again. A reader that was further along must not
// mistake that for "nothing new" and stall until the count catches up.
TEST(FrameRingTest, ReaderFollowsAPublisherThatRestarted) {
  const std::string name = unique_name("restart");
  proto::FrameMeta meta{};
  std::uint64_t last = 0;
  {
    proto::FrameRingWriter writer(name, tiny_spec());
    proto::FrameRingReader reader(name);
    std::vector<std::byte> pixels(reader.frame_bytes());
    for (std::uint64_t i = 1; i <= 5; ++i) {
      writer.write(meta_for(i), pattern(writer.frame_bytes(), std::byte{0x01}));
      last = reader.read_latest(last, meta, pixels);
    }
    EXPECT_EQ(last, 5U);
  }
  // the writer is gone; a new one starts the ring over
  proto::FrameRingWriter restarted(name, tiny_spec());
  proto::FrameRingReader reader(name);
  std::vector<std::byte> pixels(reader.frame_bytes());
  EXPECT_EQ(reader.read_latest(last, meta, pixels), 0U);  // nothing published yet
  restarted.write(meta_for(42), pattern(restarted.frame_bytes(), std::byte{0x02}));
  const std::uint64_t seq = reader.read_latest(last, meta, pixels);
  EXPECT_EQ(seq, 1U) << "the reader ignored a restarted publisher";
  EXPECT_EQ(meta.frame_index, 42U);
  EXPECT_EQ(pixels.front(), std::byte{0x02});
}

namespace {

// True when nobody holds the ring's lock, which is how a sweep tells a dead publisher's ring
bool ring_unlocked(const std::string& camera) {
  const int fd = ::shm_open(proto::ring_path(camera).c_str(), O_RDONLY, 0);
  if (fd < 0) {
    return false;
  }
  const bool unlocked = ::flock(fd, LOCK_EX | LOCK_NB) == 0;
  ::close(fd);
  return unlocked;
}

}  // namespace

TEST(FrameRingTest, ALiveWriterHoldsItsRingAndAKilledOneLeavesItUnheld) {
  const std::string camera = unique_name("lock");
  {
    proto::FrameRingWriter writer(camera, tiny_spec());
    EXPECT_FALSE(ring_unlocked(camera));
  }

  const pid_t child = ::fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    // no destructor runs, as when the app is killed
    auto* writer = new proto::FrameRingWriter(camera, tiny_spec());
    (void)writer;
    ::_exit(0);
  }
  int status = 0;
  ASSERT_EQ(::waitpid(child, &status, 0), child);
  EXPECT_TRUE(ring_unlocked(camera)) << "the dead writer's ring is not recognisable as stale";
  ::shm_unlink(proto::ring_path(camera).c_str());
}

// Two writers on one ring would reset it under each other and interleave their sequences
TEST(FrameRingTest, ASecondWriterOnALiveRingIsRefused) {
  const std::string camera = unique_name("second");
  {
    proto::FrameRingWriter first(camera, tiny_spec());
    first.write(meta_for(7), pattern(first.frame_bytes(), std::byte{0x07}));
    EXPECT_THROW(proto::FrameRingWriter(camera, tiny_spec()), std::runtime_error);
    proto::FrameRingReader reader(camera);
    proto::FrameMeta meta{};
    std::vector<std::byte> pixels(reader.frame_bytes());
    EXPECT_EQ(reader.read_latest(0, meta, pixels), 1U) << "the refused writer reset the ring";
    EXPECT_EQ(meta.frame_index, 7U);
  }
  EXPECT_NO_THROW(proto::FrameRingWriter(camera, tiny_spec()));
}

// A writer that closes cleanly unlinks its ring, so the next one creates a new object. A reader
// still mapping the old one would wait forever unless it can tell that it has been replaced.
TEST(FrameRingTest, ReaderNoticesItsRingWasReplaced) {
  const std::string camera = unique_name("replaced");
  std::optional<proto::FrameRingWriter> writer(std::in_place, camera, tiny_spec());
  proto::FrameRingReader reader(camera);
  EXPECT_FALSE(reader.replaced());
  writer.reset();
  EXPECT_TRUE(reader.replaced()) << "an unlinked ring was not noticed";
  writer.emplace(camera, tiny_spec());
  EXPECT_TRUE(reader.replaced()) << "a ring created again under the same name was not noticed";
  proto::FrameRingReader fresh(camera);
  EXPECT_FALSE(fresh.replaced());
}
