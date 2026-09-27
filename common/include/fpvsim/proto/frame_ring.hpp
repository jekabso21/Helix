#pragma once

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <new>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>

// Frame ring of docs/INTERFACES.md section 4: one per camera, seqlock, readers never block
namespace fpvsim::proto {

inline constexpr std::uint32_t kFrameMagic = 0x46565046;  // "FPVF"
inline constexpr std::uint16_t kFrameLayoutVersion = 1;
inline constexpr std::size_t kFrameHeaderSize = 128;
inline constexpr std::size_t kFrameSlotHeaderSize = 128;
inline constexpr std::size_t kFrameSlotTrailerSize = 8;
inline constexpr std::size_t kFrameSlotAlignment = 64;
inline constexpr std::uint32_t kMinSlotCount = 3;

enum class PixelFormat : std::uint16_t { kRgba8 = 1, kRgb8 = 2 };

constexpr std::size_t bytes_per_pixel(PixelFormat format) {
  return format == PixelFormat::kRgba8 ? 4 : 3;
}

struct FrameHeader {
  std::uint32_t magic;
  std::uint16_t layout_version;
  std::uint16_t pixel_format;
  std::uint32_t width;
  std::uint32_t height;
  std::uint32_t stride_bytes;
  std::uint32_t slot_count;
  std::uint64_t slot_size_bytes;
  std::atomic<std::uint64_t> latest_seq;
  double fps_nominal;
  std::array<std::byte, 80> reserved;
};

struct SlotHeader {
  std::atomic<std::uint64_t> seq_begin;
  std::int64_t sim_time_ns;
  std::uint64_t frame_index;
  std::array<double, 3> camera_position_ned;
  std::array<double, 4> q_ned_from_camera;
  std::array<std::byte, 48> reserved;
};

static_assert(sizeof(FrameHeader) == kFrameHeaderSize);
static_assert(offsetof(FrameHeader, width) == 8);
static_assert(offsetof(FrameHeader, slot_size_bytes) == 24);
static_assert(offsetof(FrameHeader, latest_seq) == 32);
static_assert(offsetof(FrameHeader, fps_nominal) == 40);
static_assert(sizeof(SlotHeader) == kFrameSlotHeaderSize);
static_assert(offsetof(SlotHeader, sim_time_ns) == 8);
static_assert(offsetof(SlotHeader, frame_index) == 16);
static_assert(offsetof(SlotHeader, camera_position_ned) == 24);
static_assert(offsetof(SlotHeader, q_ned_from_camera) == 48);
static_assert(std::atomic<std::uint64_t>::is_always_lock_free);

struct FrameMeta {
  std::int64_t sim_time_ns;
  std::uint64_t frame_index;
  std::array<double, 3> camera_position_ned;
  std::array<double, 4> q_ned_from_camera;
};

struct RingSpec {
  std::uint32_t width;
  std::uint32_t height;
  std::uint32_t slot_count;
  PixelFormat pixel_format;
  double fps_nominal;
};

constexpr std::size_t align_up(std::size_t value, std::size_t alignment) {
  return (value + alignment - 1) / alignment * alignment;
}

constexpr std::size_t stride_bytes(const RingSpec& spec) {
  return static_cast<std::size_t>(spec.width) * bytes_per_pixel(spec.pixel_format);
}

constexpr std::size_t slot_size_bytes(const RingSpec& spec) {
  return align_up(kFrameSlotHeaderSize + stride_bytes(spec) * spec.height + kFrameSlotTrailerSize,
                  kFrameSlotAlignment);
}

constexpr std::size_t ring_size_bytes(const RingSpec& spec) {
  return kFrameHeaderSize + slot_size_bytes(spec) * spec.slot_count;
}

inline std::string ring_path(const std::string& camera_name) { return "/fpvsim." + camera_name; }

namespace detail {

// Maps a POSIX shared memory object; unmaps on destruction, never unlinks
class Mapping {
 public:
  Mapping() = default;
  Mapping(void* base, std::size_t size) : base_(base), size_(size) {}
  Mapping(Mapping&& other) noexcept
      : base_(std::exchange(other.base_, nullptr)), size_(std::exchange(other.size_, 0)) {}
  Mapping& operator=(Mapping&& other) noexcept {
    if (this != &other) {
      unmap();
      base_ = std::exchange(other.base_, nullptr);
      size_ = std::exchange(other.size_, 0);
    }
    return *this;
  }
  Mapping(const Mapping&) = delete;
  Mapping& operator=(const Mapping&) = delete;
  ~Mapping() { unmap(); }

  [[nodiscard]] std::byte* bytes() const { return static_cast<std::byte*>(base_); }
  [[nodiscard]] std::size_t size() const { return size_; }
  [[nodiscard]] bool valid() const { return base_ != nullptr; }

 private:
  void unmap() {
    if (base_ != nullptr) {
      ::munmap(base_, size_);
      base_ = nullptr;
    }
  }
  void* base_ = nullptr;
  std::size_t size_ = 0;
};

}  // namespace detail

// Single writer. Publishes with the seqlock of section 4.3; never blocks on readers.
class FrameRingWriter {
 public:
  FrameRingWriter(const std::string& camera_name, const RingSpec& spec) : spec_(spec) {
    if (spec.slot_count < kMinSlotCount) {
      throw std::runtime_error("frame ring needs at least 3 slots");
    }
    name_ = ring_path(camera_name);
    const int fd = ::shm_open(name_.c_str(), O_CREAT | O_RDWR | O_TRUNC, 0600);
    if (fd < 0) {
      throw std::runtime_error("shm_open " + name_ + ": " + std::strerror(errno));
    }
    const std::size_t size = ring_size_bytes(spec);
    if (::ftruncate(fd, static_cast<off_t>(size)) != 0) {
      ::close(fd);
      throw std::runtime_error("ftruncate " + name_ + ": " + std::strerror(errno));
    }
    void* base = ::mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    ::close(fd);
    if (base == MAP_FAILED) {
      throw std::runtime_error("mmap " + name_ + ": " + std::strerror(errno));
    }
    map_ = detail::Mapping(base, size);
    std::memset(map_.bytes(), 0, size);
    FrameHeader& header = *std::launder(reinterpret_cast<FrameHeader*>(map_.bytes()));
    header.magic = kFrameMagic;
    header.layout_version = kFrameLayoutVersion;
    header.pixel_format = static_cast<std::uint16_t>(spec.pixel_format);
    header.width = spec.width;
    header.height = spec.height;
    header.stride_bytes = static_cast<std::uint32_t>(stride_bytes(spec));
    header.slot_count = spec.slot_count;
    header.slot_size_bytes = slot_size_bytes(spec);
    header.fps_nominal = spec.fps_nominal;
    header.latest_seq.store(0, std::memory_order_release);
  }

  ~FrameRingWriter() {
    if (map_.valid()) {
      ::shm_unlink(name_.c_str());
    }
  }
  FrameRingWriter(FrameRingWriter&&) noexcept = default;
  FrameRingWriter& operator=(FrameRingWriter&&) noexcept = default;
  FrameRingWriter(const FrameRingWriter&) = delete;
  FrameRingWriter& operator=(const FrameRingWriter&) = delete;

  [[nodiscard]] std::size_t frame_bytes() const { return stride_bytes(spec_) * spec_.height; }

  // pixels must be exactly frame_bytes(); returns the published sequence number
  std::uint64_t write(const FrameMeta& meta, std::span<const std::byte> pixels) {
    if (pixels.size() != frame_bytes()) {
      throw std::runtime_error("frame size mismatch");
    }
    FrameHeader& header = *std::launder(reinterpret_cast<FrameHeader*>(map_.bytes()));
    const std::uint64_t seq = header.latest_seq.load(std::memory_order_relaxed) + 1;
    std::byte* slot = slot_at((seq - 1) % spec_.slot_count);
    SlotHeader& slot_header = *std::launder(reinterpret_cast<SlotHeader*>(slot));
    slot_header.seq_begin.store(seq, std::memory_order_release);
    slot_header.sim_time_ns = meta.sim_time_ns;
    slot_header.frame_index = meta.frame_index;
    slot_header.camera_position_ned = meta.camera_position_ned;
    slot_header.q_ned_from_camera = meta.q_ned_from_camera;
    std::memcpy(slot + kFrameSlotHeaderSize, pixels.data(), pixels.size());
    seq_end(slot).store(seq, std::memory_order_release);
    header.latest_seq.store(seq, std::memory_order_release);
    return seq;
  }

 private:
  std::byte* slot_at(std::uint64_t index) const {
    return map_.bytes() + kFrameHeaderSize + slot_size_bytes(spec_) * index;
  }
  std::atomic<std::uint64_t>& seq_end(std::byte* slot) const {
    return *std::launder(reinterpret_cast<std::atomic<std::uint64_t>*>(
        slot + slot_size_bytes(spec_) - kFrameSlotTrailerSize));
  }

  RingSpec spec_;
  std::string name_;
  detail::Mapping map_;
};

// Read-only view. read_latest() copies the newest complete frame, or reports nothing new.
class FrameRingReader {
 public:
  explicit FrameRingReader(const std::string& camera_name) {
    const std::string name = ring_path(camera_name);
    const int fd = ::shm_open(name.c_str(), O_RDONLY, 0);
    if (fd < 0) {
      throw std::runtime_error("shm_open " + name + ": " + std::strerror(errno));
    }
    struct stat info{};
    if (::fstat(fd, &info) != 0 || static_cast<std::size_t>(info.st_size) < kFrameHeaderSize) {
      ::close(fd);
      throw std::runtime_error("frame ring " + name + " is too small");
    }
    const auto size = static_cast<std::size_t>(info.st_size);
    void* base = ::mmap(nullptr, size, PROT_READ, MAP_SHARED, fd, 0);
    ::close(fd);
    if (base == MAP_FAILED) {
      throw std::runtime_error("mmap " + name + ": " + std::strerror(errno));
    }
    map_ = detail::Mapping(base, size);
    const FrameHeader& header = this->header();
    if (header.magic != kFrameMagic || header.layout_version != kFrameLayoutVersion) {
      throw std::runtime_error("frame ring " + name + " has an unknown layout");
    }
    if (ring_size_bytes(spec()) > size) {
      throw std::runtime_error("frame ring " + name + " is shorter than its header claims");
    }
  }

  [[nodiscard]] RingSpec spec() const {
    const FrameHeader& h = header();
    return RingSpec{.width = h.width,
                    .height = h.height,
                    .slot_count = h.slot_count,
                    .pixel_format = static_cast<PixelFormat>(h.pixel_format),
                    .fps_nominal = h.fps_nominal};
  }
  [[nodiscard]] std::size_t frame_bytes() const {
    return header().stride_bytes * static_cast<std::size_t>(header().height);
  }
  [[nodiscard]] std::uint64_t latest_seq() const {
    return header().latest_seq.load(std::memory_order_acquire);
  }

  // Copies the newest frame if it is newer than after_seq. Returns its sequence, or 0 when there
  // is nothing new or the writer overwrote the slot while it was being read.
  std::uint64_t read_latest(std::uint64_t after_seq, FrameMeta& meta, std::span<std::byte> pixels) {
    const FrameHeader& h = header();
    const std::uint64_t seq = h.latest_seq.load(std::memory_order_acquire);
    if (seq == 0 || seq <= after_seq || pixels.size() != frame_bytes()) {
      return 0;
    }
    const std::byte* slot = slot_at((seq - 1) % h.slot_count);
    const auto& slot_header = *std::launder(reinterpret_cast<const SlotHeader*>(slot));
    if (seq_end(slot).load(std::memory_order_acquire) != seq) {
      return 0;
    }
    meta.sim_time_ns = slot_header.sim_time_ns;
    meta.frame_index = slot_header.frame_index;
    meta.camera_position_ned = slot_header.camera_position_ned;
    meta.q_ned_from_camera = slot_header.q_ned_from_camera;
    std::memcpy(pixels.data(), slot + kFrameSlotHeaderSize, pixels.size());
    std::atomic_thread_fence(std::memory_order_acquire);
    return slot_header.seq_begin.load(std::memory_order_acquire) == seq ? seq : 0;
  }

 private:
  [[nodiscard]] const FrameHeader& header() const {
    return *std::launder(reinterpret_cast<const FrameHeader*>(map_.bytes()));
  }
  [[nodiscard]] const std::byte* slot_at(std::uint64_t index) const {
    return map_.bytes() + kFrameHeaderSize + header().slot_size_bytes * index;
  }
  [[nodiscard]] const std::atomic<std::uint64_t>& seq_end(const std::byte* slot) const {
    return *std::launder(reinterpret_cast<const std::atomic<std::uint64_t>*>(
        slot + header().slot_size_bytes - kFrameSlotTrailerSize));
  }

  detail::Mapping map_;
};

}  // namespace fpvsim::proto
