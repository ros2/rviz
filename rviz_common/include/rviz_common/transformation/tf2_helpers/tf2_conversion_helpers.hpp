// Copyright (c) 2019, Open Source Robotics Foundation, Inc.
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


#ifndef RVIZ_COMMON__TRANSFORMATION__TF2_HELPERS__TF2_CONVERSION_HELPERS_HPP_
#define RVIZ_COMMON__TRANSFORMATION__TF2_HELPERS__TF2_CONVERSION_HELPERS_HPP_

#include <string>

#include "rclcpp/time.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2/time.hpp"

#include "rviz_common/visibility_control.hpp"

namespace rviz_common
{
namespace transformation
{
namespace tf2_helpers
{

RVIZ_COMMON_PUBLIC
std_msgs::msg::Header
createHeader(const tf2::TimePoint & tf2_time, const std::string & frame_id);

RVIZ_COMMON_PUBLIC
rclcpp::Time
fromTf2TimePoint(const tf2::TimePoint & tf2_time);

RVIZ_COMMON_PUBLIC
tf2::TimePoint
toTf2TimePoint(const rclcpp::Time & time);

}  // namespace tf2_helpers
}  // namespace transformation
}  // namespace rviz_common

#endif  // RVIZ_COMMON__TRANSFORMATION__TF2_HELPERS__TF2_CONVERSION_HELPERS_HPP_
