#include <gtest/gtest.h>

#include <cstdint>
#include <thread>

#include <fpvsim/spsc_queue.hpp>

TEST(SpscQueueTest, ReportsFullAtCapacity) {
  fpvsim::SpscQueue<int, 4> queue;
  for (int i = 0; i < 4; ++i) {
    EXPECT_TRUE(queue.try_push(i));
  }
  EXPECT_FALSE(queue.try_push(99));
  EXPECT_EQ(queue.size(), 4U);
}

TEST(SpscQueueTest, PopsInPushOrderThenReportsEmpty) {
  fpvsim::SpscQueue<int, 4> queue;
  for (int i = 0; i < 4; ++i) {
    ASSERT_TRUE(queue.try_push(i));
  }
  int value = -1;
  for (int i = 0; i < 4; ++i) {
    ASSERT_TRUE(queue.try_pop(value));
    EXPECT_EQ(value, i);
  }
  EXPECT_FALSE(queue.try_pop(value));
}

TEST(SpscQueueTest, WrapsAroundManyTimes) {
  fpvsim::SpscQueue<int, 2> queue;
  int value = 0;
  for (int i = 0; i < 1000; ++i) {
    ASSERT_TRUE(queue.try_push(i));
    ASSERT_TRUE(queue.try_pop(value));
    EXPECT_EQ(value, i);
  }
}

TEST(SpscQueueTest, TransfersEveryItemBetweenTwoThreads) {
  constexpr std::uint64_t kCount = 1'000'000;
  fpvsim::SpscQueue<std::uint64_t, 1024> queue;
  std::uint64_t sum = 0;
  std::uint64_t received = 0;
  std::jthread consumer([&] {
    std::uint64_t value = 0;
    while (received < kCount) {
      if (queue.try_pop(value)) {
        sum += value;
        ++received;
      }
    }
  });
  for (std::uint64_t i = 1; i <= kCount; ++i) {
    while (!queue.try_push(i)) {
    }
  }
  consumer.join();
  EXPECT_EQ(received, kCount);
  EXPECT_EQ(sum, kCount * (kCount + 1) / 2);
}
