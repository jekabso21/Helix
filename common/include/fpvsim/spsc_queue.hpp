#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <type_traits>

namespace fpvsim {

// Single-producer single-consumer ring; Capacity must be a power of two
template <typename T, std::size_t Capacity>
class SpscQueue {
  static_assert(std::is_trivially_copyable_v<T>);
  static_assert(Capacity > 0 && (Capacity & (Capacity - 1)) == 0);

 public:
  bool try_push(const T& value) {
    const std::size_t head = head_.load(std::memory_order_relaxed);
    if (head - tail_.load(std::memory_order_acquire) == Capacity) {
      return false;
    }
    buffer_[head & kMask] = value;
    head_.store(head + 1, std::memory_order_release);
    return true;
  }

  bool try_pop(T& out) {
    const std::size_t tail = tail_.load(std::memory_order_relaxed);
    if (head_.load(std::memory_order_acquire) == tail) {
      return false;
    }
    out = buffer_[tail & kMask];
    tail_.store(tail + 1, std::memory_order_release);
    return true;
  }

  [[nodiscard]] std::size_t size() const {
    return head_.load(std::memory_order_acquire) - tail_.load(std::memory_order_acquire);
  }

 private:
  static constexpr std::size_t kMask = Capacity - 1;
  std::array<T, Capacity> buffer_{};
  alignas(64) std::atomic<std::size_t> head_{0};
  alignas(64) std::atomic<std::size_t> tail_{0};
};

}  // namespace fpvsim
