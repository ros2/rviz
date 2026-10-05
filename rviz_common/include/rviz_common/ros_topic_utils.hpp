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

#ifndef RVIZ_COMMON__ROS_TOPIC_UTILS_HPP_
#define RVIZ_COMMON__ROS_TOPIC_UTILS_HPP_

#include <string>

#include "rviz_common/visibility_control.hpp"

namespace rviz_common
{

/// Return true if the given topic or service name is hidden.
/**
 * Following the ROS 2 naming conventions, a name is hidden if any of its
 * '/'-separated tokens starts with an underscore, e.g. the internal
 * "<topic>/_buf_cpu" channels created by buffer-aware rmw implementations or
 * the "<action>/_action/feedback" topics used by actions.
 * This mirrors rclpy's topic_or_service_is_hidden(), which is what
 * "ros2 topic list" uses to hide such names by default.
 */
RVIZ_COMMON_PUBLIC
bool isTopicOrServiceHidden(const std::string & name);

}  // namespace rviz_common

#endif  // RVIZ_COMMON__ROS_TOPIC_UTILS_HPP_
