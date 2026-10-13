// Copyright (c) 2026, John C. Furey
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


#include <gmock/gmock.h>

#include <chrono>
#include <csignal>
#include <cstdlib>
#include <thread>

#include "rclcpp/utilities.hpp"
#include "rviz_common/ros_integration/ros_client_abstraction.hpp"

#include "../src/rviz_common/ros_integration/signal_handlers.hpp"

using rviz_common::ros_integration::RosClientAbstraction;

namespace
{
// Stands in for a signal handler installed by the application hosting RViz.
class IgnoredSignal
{
public:
  explicit IgnoredSignal(int signal)
  : signal_(signal), previous_(std::signal(signal, SIG_IGN)) {}

  ~IgnoredSignal()
  {
    std::signal(signal_, previous_);
  }

private:
  int signal_;
  void (*previous_)(int);
};
}  // namespace

class RosClientAbstractionSignalTest : public testing::TestWithParam<int>
{
protected:
  void TearDown() override
  {
    client_.shutdown();
  }

  RosClientAbstraction client_;
};

TEST_P(RosClientAbstractionSignalTest, signal_requests_exit_without_shutting_down_ros) {
  auto node = client_.init(0, nullptr, "rviz_signal_test", false);
  ASSERT_TRUE(client_.ok());

  ASSERT_EQ(0, std::raise(GetParam()));

  EXPECT_FALSE(client_.ok());
  EXPECT_TRUE(rclcpp::ok());
  EXPECT_FALSE(node.expired());

  client_.shutdown();
  EXPECT_FALSE(rclcpp::ok());
}

TEST_P(RosClientAbstractionSignalTest, repeated_signal_is_absorbed_until_handlers_are_restored) {
#ifdef _WIN32
  GTEST_SKIP() << "Windows resets a signal handler before calling it";
#else
  // A terminal and a non-interactive launch file can both deliver the same request.
  client_.init(0, nullptr, "rviz_signal_test", false);
  ASSERT_EQ(0, std::raise(GetParam()));
  ASSERT_EQ(0, std::raise(GetParam()));

  EXPECT_FALSE(client_.ok());
  EXPECT_TRUE(rclcpp::ok());
#endif
}

TEST_P(RosClientAbstractionSignalTest, repeated_signal_ends_an_exit_that_is_stuck) {
#ifdef _WIN32
  GTEST_SKIP() << "Windows resets a signal handler before calling it";
#else
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  const int signal = GetParam();
  EXPECT_EXIT(
  {
    RosClientAbstraction client;
    client.init(0, nullptr, "rviz_stuck_exit_test", false);
    std::raise(signal);
    std::this_thread::sleep_for(std::chrono::milliseconds(1100));
    std::raise(signal);
    std::exit(0);
  }, testing::KilledBySignal(signal), "");
#endif
}

TEST_P(RosClientAbstractionSignalTest, restoring_handlers_returns_signals_to_the_application) {
  IgnoredSignal ignored_signal(GetParam());
  client_.init(0, nullptr, "rviz_signal_test", false);
  ASSERT_EQ(0, std::raise(GetParam()));
  ASSERT_FALSE(client_.ok());

  rviz_common::ros_integration::restoreSignalHandlers();

  EXPECT_EQ(SIG_IGN, std::signal(GetParam(), SIG_IGN));
  EXPECT_TRUE(rclcpp::ok());
}

TEST_P(RosClientAbstractionSignalTest, shutdown_restores_previous_signal_handler) {
  IgnoredSignal ignored_signal(GetParam());
  client_.init(0, nullptr, "rviz_signal_test", false);
  client_.shutdown();

  EXPECT_EQ(SIG_IGN, std::signal(GetParam(), SIG_IGN));
}

TEST_P(RosClientAbstractionSignalTest, a_new_session_clears_the_exit_request) {
  {
    RosClientAbstraction first_client;
    first_client.init(0, nullptr, "rviz_signal_test", false);
    ASSERT_EQ(0, std::raise(GetParam()));
    ASSERT_FALSE(first_client.ok());
    first_client.shutdown();
  }

  client_.init(0, nullptr, "rviz_signal_test", false);
  EXPECT_TRUE(client_.ok());
}

#ifndef _WIN32
namespace
{
int application_calls = 0;

void countSignal(int, siginfo_t *, void *)
{
  ++application_calls;
}
}  // namespace

TEST_P(RosClientAbstractionSignalTest, application_siginfo_handler_is_chained_and_restored) {
  struct sigaction application = {};
  application.sa_sigaction = countSignal;
  application.sa_flags = SA_SIGINFO;
  sigemptyset(&application.sa_mask);
  struct sigaction previous = {};
  ASSERT_EQ(0, sigaction(GetParam(), &application, &previous));
  application_calls = 0;

  client_.init(0, nullptr, "rviz_signal_test", false);
  ASSERT_EQ(0, std::raise(GetParam()));
  EXPECT_FALSE(client_.ok());
  EXPECT_EQ(1, application_calls);

  client_.shutdown();
  struct sigaction current = {};
  ASSERT_EQ(0, sigaction(GetParam(), nullptr, &current));
  EXPECT_TRUE(current.sa_flags & SA_SIGINFO);
  EXPECT_EQ(current.sa_sigaction, &countSignal);
  sigaction(GetParam(), &previous, nullptr);
}
#endif

INSTANTIATE_TEST_SUITE_P(
  InterruptAndTerminate, RosClientAbstractionSignalTest, testing::Values(SIGINT, SIGTERM));
