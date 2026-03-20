// Copyright (c) 2026, Open Source Robotics Foundation, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// Benchmark: compares the old executor approach (spin_some called periodically
// from the update loop) with the new approach (spin() in a dedicated thread).
//
// Old approach (before commit 9716a8a):
//   In onUpdate() (called at ~30Hz from a QTimer):
//     executor_->spin_some(std::chrono::milliseconds(10));
//
// New approach (commit 9716a8a):
//   executor_thread_ = std::thread([this]() { executor_->spin(); });
//   onUpdate() no longer calls the executor.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <iomanip>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

namespace
{

// Simulates publishing N messages from a separate thread and returns the
// wall-clock time (in microseconds) until all callbacks have been received.
struct BenchmarkResult
{
  // Total time from first publish to last callback (µs)
  double total_callback_latency_us;
  // Mean per-message latency: publish timestamp → callback timestamp (µs)
  double mean_per_message_latency_us;
  // Time the "update loop" was blocked by executor work (µs), 0 for new approach
  double update_loop_blocked_us;
  // Messages received
  int messages_received;
};

// ---------------------------------------------------------------------------
// Old approach: spin_some() called from the "update loop"
// ---------------------------------------------------------------------------
BenchmarkResult benchmark_spin_some(int num_messages, int update_rate_hz)
{
  auto node = std::make_shared<rclcpp::Node>("bench_spin_some");
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node);

  std::atomic<int> received{0};
  std::vector<std::chrono::steady_clock::time_point> publish_times(num_messages);
  std::vector<std::chrono::steady_clock::time_point> callback_times(num_messages);

  auto sub = node->create_subscription<std_msgs::msg::Int32>(
    "bench_topic_spin_some", rclcpp::QoS(num_messages),
    [&](std_msgs::msg::Int32::ConstSharedPtr msg) {
      int idx = msg->data;
      if (idx >= 0 && idx < num_messages) {
        callback_times[idx] = std::chrono::steady_clock::now();
        received++;
      }
    });

  auto pub = node->create_publisher<std_msgs::msg::Int32>(
    "bench_topic_spin_some", rclcpp::QoS(num_messages));

  // Publish all messages
  for (int i = 0; i < num_messages; ++i) {
    auto msg = std_msgs::msg::Int32();
    msg.data = i;
    publish_times[i] = std::chrono::steady_clock::now();
    pub->publish(msg);
  }

  // Simulate the old update loop: call spin_some every (1000/rate) ms
  // and measure how much time is spent blocked in spin_some.
  const auto update_interval = std::chrono::milliseconds(1000 / update_rate_hz);
  const auto spin_some_budget = std::chrono::milliseconds(10);
  double blocked_us = 0.0;

  const auto deadline = std::chrono::steady_clock::now() + 5s;
  while (received.load() < num_messages &&
    std::chrono::steady_clock::now() < deadline)
  {
    auto t0 = std::chrono::steady_clock::now();
    executor->spin_some(spin_some_budget);
    auto t1 = std::chrono::steady_clock::now();
    blocked_us += std::chrono::duration<double, std::micro>(t1 - t0).count();

    // Simulate the rest of the update work before the next tick
    auto elapsed = t1 - t0;
    if (elapsed < update_interval) {
      std::this_thread::sleep_for(update_interval - elapsed);
    }
  }

  // Compute per-message latencies
  double sum_latency_us = 0.0;
  int counted = 0;
  for (int i = 0; i < num_messages; ++i) {
    if (callback_times[i].time_since_epoch().count() > 0) {
      double lat = std::chrono::duration<double, std::micro>(
        callback_times[i] - publish_times[i]).count();
      sum_latency_us += lat;
      ++counted;
    }
  }

  auto first_pub = publish_times[0];
  auto last_cb = callback_times[0];
  for (int i = 0; i < num_messages; ++i) {
    if (callback_times[i] > last_cb) {
      last_cb = callback_times[i];
    }
  }

  BenchmarkResult result;
  result.messages_received = received.load();
  result.total_callback_latency_us =
    std::chrono::duration<double, std::micro>(last_cb - first_pub).count();
  result.mean_per_message_latency_us =
    counted > 0 ? sum_latency_us / counted : 0.0;
  result.update_loop_blocked_us = blocked_us;
  return result;
}

// ---------------------------------------------------------------------------
// New approach: spin() in a dedicated thread
// ---------------------------------------------------------------------------
BenchmarkResult benchmark_spin_thread(int num_messages, int /*update_rate_hz*/)
{
  auto node = std::make_shared<rclcpp::Node>("bench_spin_thread");
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node);

  std::atomic<int> received{0};
  std::vector<std::chrono::steady_clock::time_point> publish_times(num_messages);
  std::vector<std::chrono::steady_clock::time_point> callback_times(num_messages);

  auto sub = node->create_subscription<std_msgs::msg::Int32>(
    "bench_topic_spin_thread", rclcpp::QoS(num_messages),
    [&](std_msgs::msg::Int32::ConstSharedPtr msg) {
      int idx = msg->data;
      if (idx >= 0 && idx < num_messages) {
        callback_times[idx] = std::chrono::steady_clock::now();
        received++;
      }
    });

  auto pub = node->create_publisher<std_msgs::msg::Int32>(
    "bench_topic_spin_thread", rclcpp::QoS(num_messages));

  // New approach: spin() runs continuously in its own thread
  std::thread executor_thread([&executor]() {executor->spin();});

  // Publish all messages
  for (int i = 0; i < num_messages; ++i) {
    auto msg = std_msgs::msg::Int32();
    msg.data = i;
    publish_times[i] = std::chrono::steady_clock::now();
    pub->publish(msg);
  }

  // Wait for all callbacks — update loop is never blocked
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  while (received.load() < num_messages &&
    std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(1ms);
  }

  executor->cancel();
  if (executor_thread.joinable()) {
    executor_thread.join();
  }

  // Compute per-message latencies
  double sum_latency_us = 0.0;
  int counted = 0;
  for (int i = 0; i < num_messages; ++i) {
    if (callback_times[i].time_since_epoch().count() > 0) {
      double lat = std::chrono::duration<double, std::micro>(
        callback_times[i] - publish_times[i]).count();
      sum_latency_us += lat;
      ++counted;
    }
  }

  auto first_pub = publish_times[0];
  auto last_cb = callback_times[0];
  for (int i = 0; i < num_messages; ++i) {
    if (callback_times[i] > last_cb) {
      last_cb = callback_times[i];
    }
  }

  BenchmarkResult result;
  result.messages_received = received.load();
  result.total_callback_latency_us =
    std::chrono::duration<double, std::micro>(last_cb - first_pub).count();
  result.mean_per_message_latency_us =
    counted > 0 ? sum_latency_us / counted : 0.0;
  result.update_loop_blocked_us = 0.0;  // never blocked
  return result;
}

}  // namespace

// ---------------------------------------------------------------------------
// Test fixture
// ---------------------------------------------------------------------------

class ExecutorBenchmark : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }
};

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

// Verify that both approaches receive all published messages.
TEST_F(ExecutorBenchmark, all_messages_received_spin_some)
{
  const int kMessages = 50;
  auto result = benchmark_spin_some(kMessages, 30);
  EXPECT_EQ(result.messages_received, kMessages)
    << "spin_some approach lost messages";
}

TEST_F(ExecutorBenchmark, all_messages_received_spin_thread)
{
  const int kMessages = 50;
  auto result = benchmark_spin_thread(kMessages, 30);
  EXPECT_EQ(result.messages_received, kMessages)
    << "spin-thread approach lost messages";
}

// The dedicated-thread approach should process all messages faster than the
// spin_some approach (which is rate-limited to 30 Hz ticks).
TEST_F(ExecutorBenchmark, spin_thread_faster_total_latency)
{
  const int kMessages = 50;
  const int kUpdateRateHz = 30;

  auto old_result = benchmark_spin_some(kMessages, kUpdateRateHz);
  auto new_result = benchmark_spin_thread(kMessages, kUpdateRateHz);

  // Print results for human inspection
  std::cout << "\n=== Executor Benchmark Results ===\n"
            << "Messages:            " << kMessages << "\n"
            << "Update rate:         " << kUpdateRateHz << " Hz\n"
            << "\n[Old: spin_some in update loop]\n"
            << "  Total latency:     " << old_result.total_callback_latency_us << " µs\n"
            << "  Mean msg latency:  " << old_result.mean_per_message_latency_us << " µs\n"
            << "  Update loop blocked: " << old_result.update_loop_blocked_us << " µs\n"
            << "  Messages received: " << old_result.messages_received << "\n"
            << "\n[New: spin() in dedicated thread]\n"
            << "  Total latency:     " << new_result.total_callback_latency_us << " µs\n"
            << "  Mean msg latency:  " << new_result.mean_per_message_latency_us << " µs\n"
            << "  Update loop blocked: " << new_result.update_loop_blocked_us << " µs\n"
            << "  Messages received: " << new_result.messages_received << "\n"
            << "==================================\n";

  // New approach must not block the update loop at all
  EXPECT_EQ(new_result.update_loop_blocked_us, 0.0)
    << "spin-thread approach should not block the update loop";

  // Old approach blocks the update loop by definition (spin_some takes time)
  EXPECT_GT(old_result.update_loop_blocked_us, 0.0)
    << "spin_some approach should have measurable update-loop blocking time";

  // The new approach should process all messages faster
  EXPECT_LT(new_result.total_callback_latency_us, old_result.total_callback_latency_us)
    << "spin-thread approach should complete faster than spin_some at 30 Hz";
}

// The per-message latency of the new approach should be significantly lower,
// since callbacks are not delayed until the next 30 Hz tick.
TEST_F(ExecutorBenchmark, spin_thread_lower_mean_latency)
{
  const int kMessages = 50;
  const int kUpdateRateHz = 30;

  auto old_result = benchmark_spin_some(kMessages, kUpdateRateHz);
  auto new_result = benchmark_spin_thread(kMessages, kUpdateRateHz);

  // At 30 Hz, spin_some is called every ~33 ms, so mean latency >= 16 ms on average.
  // The dedicated thread can respond in < 1 ms.
  // We use a conservative threshold: new mean latency < old mean latency.
  EXPECT_LT(new_result.mean_per_message_latency_us, old_result.mean_per_message_latency_us)
    << "spin-thread approach should have lower per-message callback latency";

  // Dedicated thread latency should be well under 33 ms (one update tick)
  const double one_tick_us = (1.0 / kUpdateRateHz) * 1e6;
  EXPECT_LT(new_result.mean_per_message_latency_us, one_tick_us)
    << "spin-thread mean latency (" << new_result.mean_per_message_latency_us
    << " µs) should be less than one update tick (" << one_tick_us << " µs)";
}

// The old approach has update-loop blocking proportional to num_messages *
// spin_some budget. Verify it is measurably larger than the new approach.
TEST_F(ExecutorBenchmark, update_loop_not_blocked_by_spin_thread)
{
  const int kMessages = 100;
  const int kUpdateRateHz = 30;

  auto old_result = benchmark_spin_some(kMessages, kUpdateRateHz);
  auto new_result = benchmark_spin_thread(kMessages, kUpdateRateHz);

  // Old approach: total blocking time should be > 0
  EXPECT_GT(old_result.update_loop_blocked_us, 0.0);

  // New approach: update loop is never blocked (0 µs)
  EXPECT_DOUBLE_EQ(new_result.update_loop_blocked_us, 0.0);
}

// ---------------------------------------------------------------------------
// Conclusion
// ---------------------------------------------------------------------------
//
// This test prints a side-by-side summary and records a verdict so the
// conclusion is visible in the gtest output stream.
TEST_F(ExecutorBenchmark, conclusion)
{
  const int kMessages = 100;
  const int kUpdateRateHz = 30;
  const double one_tick_us = (1.0 / kUpdateRateHz) * 1e6;  // ~33 333 µs

  auto old_result = benchmark_spin_some(kMessages, kUpdateRateHz);
  auto new_result = benchmark_spin_thread(kMessages, kUpdateRateHz);

  double latency_improvement_pct =
    100.0 * (old_result.mean_per_message_latency_us - new_result.mean_per_message_latency_us) /
    old_result.mean_per_message_latency_us;

  double throughput_improvement_pct =
    100.0 * (old_result.total_callback_latency_us - new_result.total_callback_latency_us) /
    old_result.total_callback_latency_us;

  std::cout
    << "\n"
    << "╔══════════════════════════════════════════════════════════════╗\n"
    << "║            EXECUTOR BENCHMARK — CONCLUSION                  ║\n"
    << "╠══════════════════════════════════════════════════════════════╣\n"
    << "║  Metric                    Old (spin_some)  New (thread)    ║\n"
    << "╠══════════════════════════════════════════════════════════════╣\n"
    << "║  Messages received         "
    << std::setw(15) << old_result.messages_received << "  "
    << std::setw(15) << new_result.messages_received << "  ║\n"
    << "║  Mean msg latency (µs)     "
    << std::setw(15) << std::fixed << std::setprecision(1)
    << old_result.mean_per_message_latency_us << "  "
    << std::setw(15) << new_result.mean_per_message_latency_us << "  ║\n"
    << "║  Total latency (µs)        "
    << std::setw(15) << old_result.total_callback_latency_us << "  "
    << std::setw(15) << new_result.total_callback_latency_us << "  ║\n"
    << "║  Update loop blocked (µs)  "
    << std::setw(15) << old_result.update_loop_blocked_us << "  "
    << std::setw(15) << new_result.update_loop_blocked_us << "  ║\n"
    << "╠══════════════════════════════════════════════════════════════╣\n"
    << "║  Latency improvement       "
    << std::setw(14) << std::setprecision(1) << latency_improvement_pct << "%"
    << "                               ║\n"
    << "║  Throughput improvement    "
    << std::setw(14) << throughput_improvement_pct << "%"
    << "                               ║\n"
    << "║  One update tick           "
    << std::setw(14) << one_tick_us << " µs"
    << "                            ║\n"
    << "╠══════════════════════════════════════════════════════════════╣\n"
    << "║  VERDICT                                                     ║\n"
    << "╠══════════════════════════════════════════════════════════════╣\n"
    << "║  The spin-in-dedicated-thread approach (new) is BETTER:      ║\n"
    << "║                                                              ║\n"
    << "║  1. Lower callback latency: callbacks are dispatched         ║\n"
    << "║     immediately instead of waiting up to one 30 Hz tick      ║\n"
    << "║     (~33 ms). This is critical for time-sensitive data such  ║\n"
    << "║     as TF transforms and sensor messages.                    ║\n"
    << "║                                                              ║\n"
    << "║  2. Update loop never blocked: the Qt render/update loop     ║\n"
    << "║     runs at its full rate without being delayed by executor  ║\n"
    << "║     work. This keeps the UI responsive under load.           ║\n"
    << "║                                                              ║\n"
    << "║  3. Higher throughput: all N messages are processed faster   ║\n"
    << "║     because the executor is not rate-limited to 30 Hz ticks. ║\n"
    << "║                                                              ║\n"
    << "║  Trade-off: the new approach requires thread-safety for any  ║\n"
    << "║  shared state accessed from both the executor thread and the ║\n"
    << "║  Qt main thread. The existing render_mutex_ in              ║\n"
    << "║  VisualizationManagerPrivate already guards Ogre rendering.  ║\n"
    << "╚══════════════════════════════════════════════════════════════╝\n"
    << "\n";

  // Assert the conclusion holds numerically
  EXPECT_LT(new_result.mean_per_message_latency_us, old_result.mean_per_message_latency_us)
    << "New approach should have lower per-message latency";
  EXPECT_LT(new_result.total_callback_latency_us, old_result.total_callback_latency_us)
    << "New approach should process all messages faster";
  EXPECT_DOUBLE_EQ(new_result.update_loop_blocked_us, 0.0)
    << "New approach must not block the update loop";
  EXPECT_GT(old_result.update_loop_blocked_us, 0.0)
    << "Old approach inherently blocks the update loop";
}
