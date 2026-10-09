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


#ifndef RVIZ_COMMON__ROS_INTEGRATION__SIGNAL_HANDLERS_HPP_
#define RVIZ_COMMON__ROS_INTEGRATION__SIGNAL_HANDLERS_HPP_

#include "rviz_common/visibility_control.hpp"

namespace rviz_common
{
namespace ros_integration
{

/// Handle SIGINT and SIGTERM by requesting exit instead of shutting ROS down.
/**
 * Handlers installed by the application are called after RViz records the request.
 * Throws std::runtime_error if the handlers cannot be installed.
 */
void installSignalHandlers();

/// Whether SIGINT or SIGTERM arrived since installSignalHandlers().
bool exitRequested();

/// Put back the handlers that installSignalHandlers() replaced, if they are still replaced.
RVIZ_COMMON_PUBLIC
void restoreSignalHandlers();

}  // namespace ros_integration
}  // namespace rviz_common

#endif  // RVIZ_COMMON__ROS_INTEGRATION__SIGNAL_HANDLERS_HPP_
