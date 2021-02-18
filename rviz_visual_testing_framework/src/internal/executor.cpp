/*
 * Copyright (c) 2018, Bosch Software Innovations GmbH.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its contributors
 *       may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "rviz_visual_testing_framework/internal/executor.hpp"

#include <QTimer>  // NOLINT

Executor::Executor()
: default_delay_interval_(2000)
{
  total_delay_ = default_delay_interval_;
}

void Executor::queueAction(std::function<void(void)> action)
{
  // The static method QTimer::singleShot allocates an object inside of it,
  // which clang static analysis flags as a potential memory leak.  However,
  // the allocation becomes part of the Qt object hierarchy attached to this
  // object, so it is in fact not a leak.  Just make clang static analysis
  // skip this line of code.
#ifndef __clang_analyzer__
  QTimer::singleShot(total_delay_, this, action);
#endif
  increaseTotalDelay();
}

void Executor::wait(size_t milliseconds_to_wait)
{
  total_delay_ += static_cast<int>(milliseconds_to_wait);
}

void Executor::increaseTotalDelay()
{
  total_delay_ += default_delay_interval_;
}
