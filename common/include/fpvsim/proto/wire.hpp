#pragma once

#include <bit>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <type_traits>

// Explicit little-endian field serialization; never memcpy whole structs
namespace fpvsim::proto {

static_assert(std::endian::native == std::endian::little,
              "wire format assumes a little-endian host");

class Writer {
 public:
  explicit Writer(std::span<std::byte> buffer) : buffer_(buffer) {}

  template <typename T>
  void put(T value) {
    static_assert(std::is_trivially_copyable_v<T>);
    if (offset_ + sizeof(T) <= buffer_.size()) {
      std::memcpy(buffer_.data() + offset_, &value, sizeof(T));
    }
    offset_ += sizeof(T);
  }

  void put_f64(double value) { put(std::bit_cast<std::uint64_t>(value)); }
  void put_f32(float value) { put(std::bit_cast<std::uint32_t>(value)); }

  [[nodiscard]] std::size_t offset() const { return offset_; }
  [[nodiscard]] bool overflowed() const { return offset_ > buffer_.size(); }

 private:
  std::span<std::byte> buffer_;
  std::size_t offset_ = 0;
};

class Reader {
 public:
  explicit Reader(std::span<const std::byte> buffer) : buffer_(buffer) {}

  template <typename T>
  T get() {
    static_assert(std::is_trivially_copyable_v<T>);
    T value{};
    if (offset_ + sizeof(T) <= buffer_.size()) {
      std::memcpy(&value, buffer_.data() + offset_, sizeof(T));
    } else {
      failed_ = true;
    }
    offset_ += sizeof(T);
    return value;
  }

  double get_f64() { return std::bit_cast<double>(get<std::uint64_t>()); }
  float get_f32() { return std::bit_cast<float>(get<std::uint32_t>()); }

  [[nodiscard]] std::size_t offset() const { return offset_; }
  [[nodiscard]] bool failed() const { return failed_; }

 private:
  std::span<const std::byte> buffer_;
  std::size_t offset_ = 0;
  bool failed_ = false;
};

}  // namespace fpvsim::proto
