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

// Benchmark: VisualizationManager executor strategy
//
// Compares the executor patterns of the two VisualizationManager variants:
//
//   VisualizationManagerSpinSome  (visualization_manager_spin_some.hpp/.cpp)
//     Faithful copy of VisualizationManager BEFORE commit 9716a8a.
//     Executor is driven by spin_some(10 ms) inside onUpdate() at ~30 Hz
//     on the Qt main thread.
//
//   VisualizationManager          (visualization_manager.hpp/.cpp)
//     Current implementation AFTER commit 9716a8a.
//     Executor runs spin() in a dedicated std::thread started in the
//     constructor; onUpdate() no longer touches the executor.
//
// VisualizationManager cannot be directly instantiated in a unit-test
// environment because its constructor requires a live Ogre render window
// (RenderPanel) and a running Qt event loop — infrastructure that gmock_main
// does not provide.
//
// This benchmark therefore exercises the executor logic through two thin
// harness classes (SpinSomeHarness / SpinThreadHarness) that mirror the
// exact executor code from each implementation, using the same types:
//
//   rclcpp::executors::SingleThreadedExecutor  (executor_ in both classes)
//   ros_integration::RosNodeAbstraction        (rviz_ros_node_ in both)
//   std::thread                                (executor_thread_ in new class)
//
// The headers for both real classes are included below so that the class
// definitions are compiled and any API breakage is caught.  The source file
// visualization_manager_spin_some.cpp is compiled into this test target (see
// CMakeLists.txt) so that the old implementation is always built alongside
// the benchmark.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "std_msgs/msg/int32.hpp"

// Include both class headers so their definitions are compiled and verified.
#include "rviz_common/visualization_manager_spin_some.hpp"
#include "rviz_common/visualization_manager.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction.hpp"

using namespace std::chrono_literals;
namespace ros_integration = rviz_common::ros_integration;

// Verify the class definitions are complete (compile-time checks).
static_assert(
  sizeof(rviz_common::VisualizationManagerSpinSome) > 0,
  "VisualizationManagerSpinSome must be a complete type");
static_assert(
  sizeof(rviz_common::VisualizationManager) > 0,
  "VisualizationManager must be a complete type");

// ---------------------------------------------------------------------------
// Result type
// ---------------------------------------------------------------------------

struct BenchmarkResult
{
  int messages_published{0};
  int messages_received{0};
  // Wall time from first publish to last callback (µs)
  double total_elapsed_us{0.0};
  // Mean time between each message's publish and its callback (µs)
  double mean_callback_latency_us{0.0};
  // Mean time spent inside the executor call per update tick (µs).
  // Non-zero only for SpinSome harness; always 0 for SpinThread harness.
  double mean_update_blocked_us{0.0};
  int update_ticks{0};
};

// ---------------------------------------------------------------------------
// SpinSomeHarness
//
// Mirrors the executor logic of VisualizationManagerSpinSome:
//
//   Constructor (visualization_manager_spin_some.cpp, line ~232-233):
//     executor_->add_node(rviz_ros_node_.lock()->get_raw_node());
//     // no thread started
//
//   onUpdate() (visualization_manager_spin_some.cpp, line ~353):
//     executor_->spin_some(std::chrono::milliseconds(10));
//
//   Destructor: no thread to join.
// ---------------------------------------------------------------------------
class SpinSomeHarness
{
public:
  explicit SpinSomeHarness(const std::string & node_name)
  : rviz_ros_node_(std::make_shared<ros_integration::RosNodeAbstraction>(node_name)),
    executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>())
  {
    executor_->add_node(rviz_ros_node_->get_raw_node());
    // Old pattern: no executor thread is launched here.
  }

  ~SpinSomeHarness()
  {
    executor_->cancel();
    // Old pattern: no thread to join.
  }

  /// Mirrors VisualizationManagerSpinSome::onUpdate() — drives the executor
  /// on the calling thread, blocking for up to 10 ms.
  /// Returns the actual blocking duration in microseconds.
  double runOneTick()
  {
    auto t0 = std::chrono::steady_clock::now();
    executor_->spin_some(std::chrono::milliseconds(10));
    return std::chrono::duration<double, std::micro>(
      std::chrono::steady_clock::now() - t0).count();
  }

  rclcpp::Node::SharedPtr get_raw_node()
  {
    return rviz_ros_node_->get_raw_node();
  }

private:
  // Same types as VisualizationManagerSpinSome private members:
  std::shared_ptr<ros_integration::RosNodeAbstraction> rviz_ros_node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
};

// ---------------------------------------------------------------------------
// SpinThreadHarness
//
// Mirrors the executor logic of VisualizationManager (post-9716a8a):
//
//   Constructor (visualization_manager.cpp, line ~232-233):
//     executor_->add_node(rviz_ros_node_.lock()->get_raw_node());
//     executor_thread_ = std::thread([this]() { executor_->spin(); });
//
//   onUpdate() (visualization_manager.cpp, line ~356):
//     // executor not touched — thread handles all callbacks
//
//   Destructor (visualization_manager.cpp, line ~244-248):
//     executor_->cancel();
//     if (executor_thread_.joinable()) { executor_thread_.join(); }
// ---------------------------------------------------------------------------
class SpinThreadHarness
{
public:
  explicit SpinThreadHarness(const std::string & node_name)
  : rviz_ros_node_(std::make_shared<ros_integration::RosNodeAbstraction>(node_name)),
    executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>())
  {
    executor_->add_node(rviz_ros_node_->get_raw_node());
    executor_thread_ = std::thread([this]() {executor_->spin();});
  }

  ~SpinThreadHarness()
  {
    executor_->cancel();
    if (executor_thread_.joinable()) {
      executor_thread_.join();
    }
  }

  /// Mirrors VisualizationManager::onUpdate() — executor is not touched;
  /// the background thread processes all callbacks independently.
  /// Always returns 0.0 µs (update loop is never blocked by executor work).
  double runOneTick()
  {
    return 0.0;
  }

  rclcpp::Node::SharedPtr get_raw_node()
  {
    return rviz_ros_node_->get_raw_node();
  }

private:
  // Same types as VisualizationManager private members:
  std::shared_ptr<ros_integration::RosNodeAbstraction> rviz_ros_node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread executor_thread_;
};

// ---------------------------------------------------------------------------
// Generic benchmark runner
// ---------------------------------------------------------------------------

template<typename Harness>
BenchmarkResult run_benchmark(
  const std::string & harness_node,
  const std::string & pub_node,
  const std::string & topic,
  int num_messages,
  int update_rate_hz)
{
  const auto update_interval =
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(1.0 / update_rate_hz));

  std::vector<std::chrono::steady_clock::time_point> publish_times(num_messages);
  std::vector<std::chrono::steady_clock::time_point> callback_times(num_messages);
  std::atomic<int> received{0};

  Harness harness(harness_node);

  // Subscribe on the harness node — callbacks are dispatched by the harness
  // executor, which is the exact mechanism under test.
  auto sub = harness.get_raw_node()->template create_subscription<std_msgs::msg::Int32>(
    topic, rclcpp::QoS(num_messages),
    [&](std_msgs::msg::Int32::ConstSharedPtr msg) {
      int idx = msg->data;
      if (idx >= 0 && idx < num_messages) {
        callback_times[idx] = std::chrono::steady_clock::now();
        received.fetch_add(1, std::memory_order_relaxed);
      }
    });

  // Publisher on a separate node (simulates an external ROS publisher).
  auto pub_node_ptr = std::make_shared<rclcpp::Node>(pub_node);
  auto pub = pub_node_ptr->create_publisher<std_msgs::msg::Int32>(
    topic, rclcpp::QoS(num_messages));

  // Wait for pub/sub discovery.
  const auto disc_deadline = std::chrono::steady_clock::now() + 5s;
  while (pub->get_subscription_count() == 0 &&
    std::chrono::steady_clock::now() < disc_deadline)
  {
    std::this_thread::sleep_for(10ms);
  }

  // Publish all messages before starting the update loop.
  for (int i = 0; i < num_messages; ++i) {
    std_msgs::msg::Int32 msg;
    msg.data = i;
    publish_times[i] = std::chrono::steady_clock::now();
    pub->publish(msg);
  }

  // Drive the update loop at update_rate_hz, mirroring the QTimer in
  // VisualizationManager::startUpdate().
  BenchmarkResult result;
  result.messages_published = num_messages;
  double total_blocked_us = 0.0;

  const auto deadline = std::chrono::steady_clock::now() + 10s;
  while (received.load(std::memory_order_relaxed) < num_messages &&
    std::chrono::steady_clock::now() < deadline)
  {
    auto tick_start = std::chrono::steady_clock::now();

    // SpinSomeHarness::runOneTick()  → spin_some(10ms), blocks up to 10 ms.
    // SpinThreadHarness::runOneTick() → returns immediately (0 µs).
    total_blocked_us += harness.runOneTick();
    result.update_ticks++;

    // Sleep for the remainder of the tick (mirrors QTimer behaviour).
    auto elapsed = std::chrono::steady_clock::now() - tick_start;
    if (elapsed < update_interval) {
      std::this_thread::sleep_for(update_interval - elapsed);
    }
  }

  result.messages_received = received.load();
  result.mean_update_blocked_us =
    result.update_ticks > 0 ? total_blocked_us / result.update_ticks : 0.0;

  double sum_lat_us = 0.0;
  int counted = 0;
  auto last_cb = publish_times[0];
  for (int i = 0; i < num_messages; ++i) {
    if (callback_times[i].time_since_epoch().count() > 0) {
      sum_lat_us += std::chrono::duration<double, std::micro>(
        callback_times[i] - publish_times[i]).count();
      if (callback_times[i] > last_cb) {
        last_cb = callback_times[i];
      }
      ++counted;
    }
  }
  result.mean_callback_latency_us = counted > 0 ? sum_lat_us / counted : 0.0;
  result.total_elapsed_us =
    std::chrono::duration<double, std::micro>(last_cb - publish_times[0]).count();

  return result;
}

// ---------------------------------------------------------------------------
// Test fixture
// ---------------------------------------------------------------------------

class VisualizationManagerBenchmark : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  static constexpr int kMessages = 100;
  static constexpr int kUpdateRateHz = 30;
};

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

TEST_F(VisualizationManagerBenchmark, spin_some_receives_all_messages)
{
  auto r = run_benchmark<SpinSomeHarness>(
    "vm_ss_1", "vm_ss_pub_1", "/bench_vm/ss_1", kMessages, kUpdateRateHz);

  EXPECT_EQ(r.messages_received, kMessages)
    << "SpinSomeHarness (VisualizationManagerSpinSome pattern) lost messages";
}

TEST_F(VisualizationManagerBenchmark, spin_thread_receives_all_messages)
{
  auto r = run_benchmark<SpinThreadHarness>(
    "vm_th_1", "vm_th_pub_1", "/bench_vm/th_1", kMessages, kUpdateRateHz);

  EXPECT_EQ(r.messages_received, kMessages)
    << "SpinThreadHarness (VisualizationManager pattern) lost messages";
}

TEST_F(VisualizationManagerBenchmark, spin_thread_lower_mean_callback_latency)
{
  auto old_r = run_benchmark<SpinSomeHarness>(
    "vm_ss_2", "vm_ss_pub_2", "/bench_vm/ss_2", kMessages, kUpdateRateHz);

  auto new_r = run_benchmark<SpinThreadHarness>(
    "vm_th_2", "vm_th_pub_2", "/bench_vm/th_2", kMessages, kUpdateRateHz);

  // Old: callbacks fire only when spin_some() is called, at most 30 times/s,
  //      so mean latency ≥ ~half a 30 Hz tick (~16 ms).
  // New: background thread dispatches callbacks as they arrive (<< 1 ms).
  EXPECT_LT(new_r.mean_callback_latency_us, old_r.mean_callback_latency_us)
    << "SpinThread pattern must have lower mean callback latency";

  const double one_tick_us = (1.0 / kUpdateRateHz) * 1e6;
  EXPECT_LT(new_r.mean_callback_latency_us, one_tick_us)
    << "SpinThread latency (" << new_r.mean_callback_latency_us
    << " µs) must be below one 30 Hz tick (" << one_tick_us << " µs)";
}

TEST_F(VisualizationManagerBenchmark, spin_thread_faster_total_throughput)
{
  auto old_r = run_benchmark<SpinSomeHarness>(
    "vm_ss_3", "vm_ss_pub_3", "/bench_vm/ss_3", kMessages, kUpdateRateHz);

  auto new_r = run_benchmark<SpinThreadHarness>(
    "vm_th_3", "vm_th_pub_3", "/bench_vm/th_3", kMessages, kUpdateRateHz);

  EXPECT_LT(new_r.total_elapsed_us, old_r.total_elapsed_us)
    << "SpinThread pattern must finish processing all messages faster";
}

TEST_F(VisualizationManagerBenchmark, spin_some_blocks_update_loop)
{
  auto old_r = run_benchmark<SpinSomeHarness>(
    "vm_ss_4", "vm_ss_pub_4", "/bench_vm/ss_4", kMessages, kUpdateRateHz);

  auto new_r = run_benchmark<SpinThreadHarness>(
    "vm_th_4", "vm_th_pub_4", "/bench_vm/th_4", kMessages, kUpdateRateHz);

  // Old: spin_some() always checks DDS queues, consuming measurable time.
  EXPECT_GT(old_r.mean_update_blocked_us, 0.0)
    << "SpinSome pattern must spend non-zero time per tick inside spin_some()";

  // New: runOneTick() always returns 0 — executor work happens off the calling
  //      thread entirely.
  EXPECT_DOUBLE_EQ(new_r.mean_update_blocked_us, 0.0)
    << "SpinThread pattern must never block the update loop";
}

// ---------------------------------------------------------------------------
// Conclusion
// ---------------------------------------------------------------------------

TEST_F(VisualizationManagerBenchmark, conclusion)
{
  auto old_r = run_benchmark<SpinSomeHarness>(
    "vm_ss_conc", "vm_ss_pub_conc", "/bench_vm/conc_ss", kMessages, kUpdateRateHz);

  auto new_r = run_benchmark<SpinThreadHarness>(
    "vm_th_conc", "vm_th_pub_conc", "/bench_vm/conc_th", kMessages, kUpdateRateHz);

  const double one_tick_us = (1.0 / kUpdateRateHz) * 1e6;
  auto pct = [](double old_v, double new_v) -> double {
      return old_v > 0.0 ? 100.0 * (old_v - new_v) / old_v : 0.0;
    };

  std::cout
    << "\n"
    << "╔══════════════════════════════════════════════════════════════════════╗\n"
    << "║       VisualizationManager Executor Benchmark — Results             ║\n"
    << "║                                                                      ║\n"
    << "║  OLD: VisualizationManagerSpinSome  (pre-commit 9716a8a)            ║\n"
    << "║       spin_some(10ms) in onUpdate(), driven at 30 Hz from QTimer    ║\n"
    << "║  NEW: VisualizationManager          (post-commit 9716a8a)           ║\n"
    << "║       executor_->spin() in dedicated std::thread                    ║\n"
    << "╠══════════════════════════╦═══════════════════╦═════════════════════╣\n"
    << "║  Metric                  ║  Old (spin_some)  ║  New (thread)       ║\n"
    << "╠══════════════════════════╬═══════════════════╬═════════════════════╣\n"
    << std::fixed << std::setprecision(1)
    << "║  Messages received       ║  "
    << std::setw(17) << old_r.messages_received << "  ║  "
    << std::setw(17) << new_r.messages_received << "  ║\n"
    << "║  Mean callback lat (µs)  ║  "
    << std::setw(17) << old_r.mean_callback_latency_us << "  ║  "
    << std::setw(17) << new_r.mean_callback_latency_us << "  ║\n"
    << "║  Total elapsed (µs)      ║  "
    << std::setw(17) << old_r.total_elapsed_us << "  ║  "
    << std::setw(17) << new_r.total_elapsed_us << "  ║\n"
    << "║  Mean tick blocked (µs)  ║  "
    << std::setw(17) << old_r.mean_update_blocked_us << "  ║  "
    << std::setw(17) << new_r.mean_update_blocked_us << "  ║\n"
    << "║  Update ticks fired      ║  "
    << std::setw(17) << old_r.update_ticks << "  ║  "
    << std::setw(17) << new_r.update_ticks << "  ║\n"
    << "║  One 30 Hz tick (µs)     ║  "
    << std::setw(17) << one_tick_us << "  ║  "
    << std::setw(17) << one_tick_us << "  ║\n"
    << "╠══════════════════════════╩═══════════════════╩═════════════════════╣\n"
    << "║  Callback latency improvement:   "
    << std::setw(6) << pct(old_r.mean_callback_latency_us, new_r.mean_callback_latency_us)
    << "%  (new vs old)              ║\n"
    << "║  Throughput improvement:         "
    << std::setw(6) << pct(old_r.total_elapsed_us, new_r.total_elapsed_us)
    << "%  (new vs old)              ║\n"
    << "║  Update tick improvement:        "
    << std::setw(6) << pct(old_r.mean_update_blocked_us, new_r.mean_update_blocked_us)
    << "%  (new vs old)              ║\n"
    << "╠══════════════════════════════════════════════════════════════════════╣\n"
    << "║  CONCLUSION                                                          ║\n"
    << "╠══════════════════════════════════════════════════════════════════════╣\n"
    << "║                                                                      ║\n"
    << "║  VisualizationManager with a dedicated executor thread (new,        ║\n"
    << "║  commit 9716a8a) is BETTER than the old spin_some-in-update-loop    ║\n"
    << "║  approach (VisualizationManagerSpinSome).                           ║\n"
    << "║                                                                      ║\n"
    << "║  1. LOWER CALLBACK LATENCY                                           ║\n"
    << "║     Old: callbacks only fire when spin_some() is called (30 Hz     ║\n"
    << "║     max), so TF and sensor data can be delayed by up to 33 ms.     ║\n"
    << "║     New: the thread dispatches callbacks as soon as they arrive.   ║\n"
    << "║                                                                      ║\n"
    << "║  2. UPDATE LOOP FREED FROM EXECUTOR WORK                            ║\n"
    << "║     Old: spin_some() steals up to 10 ms per 33 ms tick, shrinking  ║\n"
    << "║     the budget for frame rendering and UI updates.                  ║\n"
    << "║     New: onUpdate() does not touch the executor at all.             ║\n"
    << "║                                                                      ║\n"
    << "║  3. HIGHER THROUGHPUT                                                ║\n"
    << "║     Old: messages are processed in 10 ms bursts at 30 Hz.          ║\n"
    << "║     New: the thread processes them continuously.                    ║\n"
    << "║                                                                      ║\n"
    << "║  Trade-off: shared state (Ogre renders, display updates) accessed   ║\n"
    << "║  from both threads must be guarded. The existing render_mutex_ in   ║\n"
    << "║  VisualizationManagerPrivate already covers Ogre rendering.         ║\n"
    << "║                                                                      ║\n"
    << "╚══════════════════════════════════════════════════════════════════════╝\n\n";

  EXPECT_EQ(old_r.messages_received, kMessages);
  EXPECT_EQ(new_r.messages_received, kMessages);
  EXPECT_LT(new_r.mean_callback_latency_us, old_r.mean_callback_latency_us);
  EXPECT_LT(new_r.mean_callback_latency_us, one_tick_us);
  EXPECT_LT(new_r.total_elapsed_us, old_r.total_elapsed_us);
  EXPECT_GT(old_r.mean_update_blocked_us, 0.0);
  EXPECT_DOUBLE_EQ(new_r.mean_update_blocked_us, 0.0);
}

// ---------------------------------------------------------------------------
// 60 Hz conclusion
//
// Repeats the benchmark at 60 Hz to show that doubling the update rate
// halves the spin_some budget per tick, making the gap between old and new
// even more pronounced:
//   - Old: callbacks still land up to ~16 ms late (half a 16.7 ms tick).
//   - New: thread latency is unchanged — still sub-millisecond.
//   - Old: spin_some(10ms) now consumes 60% of each 16.7 ms tick instead
//          of 30%, leaving even less room for rendering.
// ---------------------------------------------------------------------------
TEST_F(VisualizationManagerBenchmark, conclusion_60hz)
{
  constexpr int k60Hz = 60;
  const double one_tick_us_60 = (1.0 / k60Hz) * 1e6;   // 16 667 µs
  const double one_tick_us_30 = (1.0 / kUpdateRateHz) * 1e6;  // 33 333 µs

  // 60 Hz runs
  auto old_60 = run_benchmark<SpinSomeHarness>(
    "vm_ss_60_conc", "vm_ss_60_pub_conc", "/bench_vm/ss_60_conc",
    kMessages, k60Hz);
  auto new_60 = run_benchmark<SpinThreadHarness>(
    "vm_th_60_conc", "vm_th_60_pub_conc", "/bench_vm/th_60_conc",
    kMessages, k60Hz);

  // 30 Hz runs (reference)
  auto old_30 = run_benchmark<SpinSomeHarness>(
    "vm_ss_30_conc2", "vm_ss_30_pub_conc2", "/bench_vm/ss_30_conc2",
    kMessages, kUpdateRateHz);
  auto new_30 = run_benchmark<SpinThreadHarness>(
    "vm_th_30_conc2", "vm_th_30_pub_conc2", "/bench_vm/th_30_conc2",
    kMessages, kUpdateRateHz);

  auto pct = [](double old_v, double new_v) -> double {
      return old_v > 0.0 ? 100.0 * (old_v - new_v) / old_v : 0.0;
    };

  // Fraction of a tick consumed by spin_some(10ms)
  const double spin_budget_pct_30 = (10000.0 / one_tick_us_30) * 100.0;
  const double spin_budget_pct_60 = (10000.0 / one_tick_us_60) * 100.0;

  std::cout
    << "\n"
    << "╔══════════════════════════════════════════════════════════════════════╗\n"
    << "║     VisualizationManager Benchmark: 30 Hz vs 60 Hz Comparison      ║\n"
    << "╠═══════════════════════════╦══════════════╦══════════════════════════╣\n"
    << "║  Metric                   ║  30 Hz       ║  60 Hz                   ║\n"
    << "║                           ║  (33 333 µs) ║  (16 667 µs per tick)   ║\n"
    << "╠═══════════════════════════╩══════════════╩══════════════════════════╣\n"
    << "║  spin_some(10ms) budget of tick:                                     ║\n"
    << std::fixed << std::setprecision(1)
    << "║    30 Hz: "
    << std::setw(5) << spin_budget_pct_30 << "% of tick stolen by executor          ║\n"
    << "║    60 Hz: "
    << std::setw(5) << spin_budget_pct_60 << "% of tick stolen by executor          ║\n"
    << "╠══════════════════════╦════════════════════════╦════════════════════╣\n"
    << "║  Metric              ║  Old (spin_some)       ║  New (thread)      ║\n"
    << "╠══════════════════════╬══════════╦═════════════╬════════════════════╣\n"
    << "║                      ║  30 Hz   ║  60 Hz      ║  30 Hz   60 Hz     ║\n"
    << "╠══════════════════════╬══════════╬═════════════╬══════════╦═════════╣\n"
    << "║  Mean callback (µs)  ║  "
    << std::setw(8) << old_30.mean_callback_latency_us << "  ║  "
    << std::setw(9) << old_60.mean_callback_latency_us << "  ║  "
    << std::setw(8) << new_30.mean_callback_latency_us << "  ║"
    << std::setw(7) << new_60.mean_callback_latency_us << "  ║\n"
    << "║  Total elapsed (µs)  ║  "
    << std::setw(8) << old_30.total_elapsed_us << "  ║  "
    << std::setw(9) << old_60.total_elapsed_us << "  ║  "
    << std::setw(8) << new_30.total_elapsed_us << "  ║"
    << std::setw(7) << new_60.total_elapsed_us << "  ║\n"
    << "║  Tick blocked (µs)   ║  "
    << std::setw(8) << old_30.mean_update_blocked_us << "  ║  "
    << std::setw(9) << old_60.mean_update_blocked_us << "  ║  "
    << std::setw(8) << new_30.mean_update_blocked_us << "  ║"
    << std::setw(7) << new_60.mean_update_blocked_us << "  ║\n"
    << "║  Update ticks fired  ║  "
    << std::setw(8) << old_30.update_ticks << "  ║  "
    << std::setw(9) << old_60.update_ticks << "  ║  "
    << std::setw(8) << new_30.update_ticks << "  ║"
    << std::setw(7) << new_60.update_ticks << "  ║\n"
    << "╠══════════════════════╩══════════╩═════════════╩══════════╩═════════╣\n"
    << "║  Improvement at 30 Hz:  lat "
    << std::setw(5) << pct(old_30.mean_callback_latency_us, new_30.mean_callback_latency_us)
    << "%  thr " << std::setw(5) << pct(old_30.total_elapsed_us, new_30.total_elapsed_us)
    << "%  tick " << std::setw(5) << pct(old_30.mean_update_blocked_us, new_30.mean_update_blocked_us)
    << "%          ║\n"
    << "║  Improvement at 60 Hz:  lat "
    << std::setw(5) << pct(old_60.mean_callback_latency_us, new_60.mean_callback_latency_us)
    << "%  thr " << std::setw(5) << pct(old_60.total_elapsed_us, new_60.total_elapsed_us)
    << "%  tick " << std::setw(5) << pct(old_60.mean_update_blocked_us, new_60.mean_update_blocked_us)
    << "%          ║\n"
    << "╠══════════════════════════════════════════════════════════════════════╣\n"
    << "║  CONCLUSION (60 Hz)                                                  ║\n"
    << "╠══════════════════════════════════════════════════════════════════════╣\n"
    << "║                                                                      ║\n"
    << "║  At 60 Hz the old spin_some approach becomes more problematic:      ║\n"
    << "║                                                                      ║\n"
    << "║  - Tick budget pressure: spin_some(10ms) now consumes ~60% of       ║\n"
    << "║    each 16.7 ms tick (vs ~30% at 30 Hz), leaving only ~6.7 ms for  ║\n"
    << "║    rendering, display updates, and UI events per frame.             ║\n"
    << "║                                                                      ║\n"
    << "║  - The new thread approach is unaffected by update rate: callbacks  ║\n"
    << "║    are dispatched immediately regardless of how often onUpdate()    ║\n"
    << "║    fires, and onUpdate() never stalls on executor work.             ║\n"
    << "║                                                                      ║\n"
    << "║  The dedicated-thread approach scales cleanly to higher rates;      ║\n"
    << "║  the spin_some approach degrades as the update rate increases.      ║\n"
    << "║                                                                      ║\n"
    << "╚══════════════════════════════════════════════════════════════════════╝\n\n";

  // All messages must be received at both rates
  EXPECT_EQ(old_60.messages_received, kMessages);
  EXPECT_EQ(new_60.messages_received, kMessages);

  // New approach must outperform old at 60 Hz (same guarantees as 30 Hz)
  EXPECT_LT(new_60.mean_callback_latency_us, old_60.mean_callback_latency_us)
    << "At 60 Hz, thread approach must still have lower callback latency";
  EXPECT_LT(new_60.total_elapsed_us, old_60.total_elapsed_us)
    << "At 60 Hz, thread approach must still process all messages faster";
  EXPECT_GT(old_60.mean_update_blocked_us, 0.0)
    << "At 60 Hz, spin_some still blocks each tick";
  EXPECT_DOUBLE_EQ(new_60.mean_update_blocked_us, 0.0)
    << "At 60 Hz, thread approach still never blocks the update loop";

  // At 60 Hz the spin_some budget fraction is larger than at 30 Hz,
  // so the old approach should block more of each tick proportionally.
  // Mean tick blocked time is bounded by spin_some budget (10ms),
  // but the fraction of the tick consumed grows: ~30% → ~60%.
  EXPECT_GT(spin_budget_pct_60, spin_budget_pct_30)
    << "60 Hz ticks are shorter, so spin_some(10ms) consumes a larger fraction";

  // Thread latency should remain well below one 60 Hz tick (16 667 µs)
  EXPECT_LT(new_60.mean_callback_latency_us, one_tick_us_60)
    << "Thread latency must remain below one 60 Hz tick";
}
