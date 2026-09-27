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


#include "signal_handlers.hpp"

#include <atomic>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <ctime>
#include <stdexcept>

namespace rviz_common
{
namespace ros_integration
{

namespace
{
static_assert(ATOMIC_BOOL_LOCK_FREE == 2, "Signal handlers require lock-free atomics");
std::atomic<bool> exit_requested{false};
constexpr int kSignals[] = {SIGINT, SIGTERM};
bool installed = false;

#ifdef _WIN32
using Action = void (*)(int);
Action previous_actions[2];

void requestExit(int signal)
{
  // Qt observes this through RosClientAbstraction::ok(); handlers must not call ROS or Qt.
  exit_requested.store(true, std::memory_order_relaxed);
  const Action previous = previous_actions[signal == SIGINT ? 0 : 1];
  if (previous != SIG_DFL && previous != SIG_IGN && previous != SIG_ERR && previous) {
    previous(signal);
  }
}
#else
struct sigaction previous_actions[2];
std::atomic<int64_t> first_request_ns{0};
static_assert(
  std::atomic<int64_t>::is_always_lock_free, "Signal handlers require lock-free atomics");
constexpr int64_t kStuckExitNs = 1000000000;

int64_t monotonicNs()
{
  timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);
  return static_cast<int64_t>(now.tv_sec) * 1000000000 + now.tv_nsec;
}

void requestExit(int signal, siginfo_t * info, void * context)
{
  // Qt observes this through RosClientAbstraction::ok(); handlers must not call ROS or Qt.
  exit_requested.store(true, std::memory_order_relaxed);
  const struct sigaction & previous = previous_actions[signal == SIGINT ? 0 : 1];
  const int64_t now = monotonicNs();
  int64_t first = 0;
  if (!first_request_ns.compare_exchange_strong(first, now) && now - first > kStuckExitNs &&
    !(previous.sa_flags & SA_SIGINFO) && previous.sa_handler == SIG_DFL)
  {
    // RViz has not exited a second after the first request, for example because a plugin
    // blocks the event loop. Take the default action, as without these handlers.
    struct sigaction action = {};
    action.sa_handler = SIG_DFL;
    sigemptyset(&action.sa_mask);
    sigaction(signal, &action, nullptr);
    raise(signal);
    return;
  }
  // Chain to a handler the application installed, as rclcpp's own handler does.
  if (previous.sa_flags & SA_SIGINFO) {
    if (previous.sa_sigaction) {
      previous.sa_sigaction(signal, info, context);
    }
  } else if (previous.sa_handler != SIG_DFL && previous.sa_handler != SIG_IGN) {
    previous.sa_handler(signal);
  }
}
#endif

void restore(size_t count)
{
  for (size_t i = 0; i < count; ++i) {
#ifdef _WIN32
    std::signal(kSignals[i], previous_actions[i]);
#else
    sigaction(kSignals[i], &previous_actions[i], nullptr);
#endif
  }
}
}  // namespace

void installSignalHandlers()
{
  restoreSignalHandlers();
  exit_requested.store(false, std::memory_order_relaxed);
#ifndef _WIN32
  first_request_ns.store(0, std::memory_order_relaxed);
#endif
  for (size_t i = 0; i < 2; ++i) {
#ifdef _WIN32
    previous_actions[i] = std::signal(kSignals[i], requestExit);
    const bool failed = previous_actions[i] == SIG_ERR;
#else
    struct sigaction action = {};
    action.sa_sigaction = requestExit;
    action.sa_flags = SA_SIGINFO | SA_RESTART;
    sigemptyset(&action.sa_mask);
    const bool failed = sigaction(kSignals[i], &action, &previous_actions[i]) != 0;
#endif
    if (failed) {
      restore(i);
      throw std::runtime_error("Failed to install RViz signal handlers");
    }
  }
  installed = true;
}

bool exitRequested()
{
  return exit_requested.load(std::memory_order_relaxed);
}

void restoreSignalHandlers()
{
  if (installed) {
    installed = false;
    restore(2);
  }
}

}  // namespace ros_integration
}  // namespace rviz_common
