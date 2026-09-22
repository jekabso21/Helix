#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace fpvsim::sim {

// Step duration statistics without allocation: fixed 10 us buckets up to 5 ms plus overflow
class StepStats {
 public:
  static constexpr double kBucketUs = 10.0;
  static constexpr std::size_t kBuckets = 500;

  void add(double step_time_us) {
    ++count_;
    sum_us_ += step_time_us;
    max_us_ = std::max(max_us_, step_time_us);
    const auto bucket = static_cast<std::size_t>(std::max(0.0, step_time_us) / kBucketUs);
    ++histogram_[std::min(bucket, kBuckets)];
  }

  [[nodiscard]] double mean_us() const {
    return count_ == 0 ? 0.0 : sum_us_ / static_cast<double>(count_);
  }
  [[nodiscard]] double max_us() const { return max_us_; }

  [[nodiscard]] double p99_us() const {
    if (count_ == 0) {
      return 0.0;
    }
    const auto target = static_cast<std::uint64_t>(std::ceil(0.99 * static_cast<double>(count_)));
    std::uint64_t seen = 0;
    for (std::size_t i = 0; i <= kBuckets; ++i) {
      seen += histogram_[i];
      if (seen >= target) {
        return static_cast<double>(i + 1) * kBucketUs;
      }
    }
    return max_us_;
  }

 private:
  std::array<std::uint64_t, kBuckets + 1> histogram_{};
  std::uint64_t count_ = 0;
  double sum_us_ = 0.0;
  double max_us_ = 0.0;
};

}  // namespace fpvsim::sim
