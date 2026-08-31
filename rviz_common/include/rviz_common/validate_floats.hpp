// Copyright (c) 2009, Willow Garage, Inc.
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


#ifndef RVIZ_COMMON__VALIDATE_FLOATS_HPP_
#define RVIZ_COMMON__VALIDATE_FLOATS_HPP_

#include <cmath>
#include <ranges>  // NOLINT(build/include_order) cpplint predates C++20 headers

#include <OgreVector.h>
#include <OgreQuaternion.h>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/color_rgba.hpp"

namespace rviz_common
{

inline bool validateFloats(float val)
{
  return !(std::isnan(val) || std::isinf(val));
}

inline bool validateFloats(double val)
{
  return !(std::isnan(val) || std::isinf(val));
}

inline bool validateFloats(const Ogre::Vector3 & vec)
{
  bool valid = true;
  valid = valid && validateFloats(vec.x);
  valid = valid && validateFloats(vec.y);
  valid = valid && validateFloats(vec.z);
  return valid;
}

inline bool validateFloats(const Ogre::Quaternion & quat)
{
  bool valid = true;
  valid = valid && validateFloats(quat.x);
  valid = valid && validateFloats(quat.y);
  valid = valid && validateFloats(quat.z);
  valid = valid && validateFloats(quat.w);
  return valid;
}

inline bool validateFloats(const geometry_msgs::msg::Point & msg)
{
  bool valid = true;
  valid = valid && validateFloats(msg.x);
  valid = valid && validateFloats(msg.y);
  valid = valid && validateFloats(msg.z);
  return valid;
}

inline bool validateFloats(const geometry_msgs::msg::Point32 & msg)
{
  bool valid = true;
  valid = valid && validateFloats(msg.x);
  valid = valid && validateFloats(msg.y);
  valid = valid && validateFloats(msg.z);
  return valid;
}

inline bool validateFloats(const geometry_msgs::msg::Vector3 & msg)
{
  bool valid = true;
  valid = valid && validateFloats(msg.x);
  valid = valid && validateFloats(msg.y);
  valid = valid && validateFloats(msg.z);
  return valid;
}

inline bool validateFloats(const geometry_msgs::msg::Twist & twist)
{
  bool valid = true;
  valid = valid && validateFloats(twist.linear);
  valid = valid && validateFloats(twist.angular);
  return valid;
}

inline bool validateFloats(const geometry_msgs::msg::Quaternion & msg)
{
  bool valid = true;
  valid = valid && validateFloats(msg.x);
  valid = valid && validateFloats(msg.y);
  valid = valid && validateFloats(msg.z);
  valid = valid && validateFloats(msg.w);
  return valid;
}

inline bool validateFloats(const std_msgs::msg::ColorRGBA & msg)
{
  bool valid = true;
  valid = valid && validateFloats(msg.r);
  valid = valid && validateFloats(msg.g);
  valid = valid && validateFloats(msg.b);
  valid = valid && validateFloats(msg.a);
  return valid;
}

inline bool validateFloats(const geometry_msgs::msg::PointStamped & msg)
{
  return validateFloats(msg.point);
}

inline bool validateFloats(const geometry_msgs::msg::Pose & msg)
{
  bool valid = true;
  valid = valid && validateFloats(msg.position);
  valid = valid && validateFloats(msg.orientation);
  return valid;
}

inline bool validateFloats(const geometry_msgs::msg::PoseStamped & msg)
{
  return validateFloats(msg.pose);
}

/// Validate any range (vector, array, span, ...) whose elements are themselves validatable.
template<typename Range>
requires std::ranges::input_range<Range>
inline bool validateFloats(const Range & range)
{
  return std::ranges::all_of(
    range, [](const auto & elem) {return validateFloats(elem);});
}

}  // namespace rviz_common

#endif  // RVIZ_COMMON__VALIDATE_FLOATS_HPP_
