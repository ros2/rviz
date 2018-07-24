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

#ifndef RVIZ_COMMON__TRANSFORMATION__STRUCTS_HPP_
#define RVIZ_COMMON__TRANSFORMATION__STRUCTS_HPP_

#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/header.hpp"

#include "rclcpp/rclcpp.hpp"

#include "rviz_common/visibility_control.hpp"
#include "rviz_common/transformation/ros_helpers/ros_conversion_helpers.hpp"

namespace rviz_common
{
namespace transformation
{

struct RVIZ_COMMON_PUBLIC Time
{
  Time()
  : seconds_(0), nanoseconds_(0) {}
  Time(uint32_t seconds, uint32_t nanoseconds)
  {
    seconds_ = seconds;
    nanoseconds_ = nanoseconds;
  }

  explicit Time(rclcpp::Time time)
  {
    nanoseconds_ = time.nanoseconds();
  }

  uint32_t seconds_;
  uint32_t nanoseconds_;
};

struct RVIZ_COMMON_PUBLIC Point
{
  Point()
  : x_(0), y_(0), z_(0) {}
  Point(float x, float y, float z)
  {
    x_ = x;
    y_ = y;
    z_ = z;
  }

  float x_;
  float y_;
  float z_;
};

struct RVIZ_COMMON_PUBLIC Quaternion
{
  Quaternion()
  : w_(1), x_(0), y_(0), z_(0) {}
  Quaternion(float w, float x, float y, float z)
  {
    w_ = w;
    x_ = x;
    y_ = y;
    z_ = z;
  }

  float w_;
  float x_;
  float y_;
  float z_;
};

struct RVIZ_COMMON_PUBLIC PoseStamped
{
  PoseStamped()
  : time_stamp_(), frame_id_(""), point_(), orientation_() {}
  explicit PoseStamped(geometry_msgs::msg::PoseStamped ros_pose)
  {
    time_stamp_.seconds_ = ros_pose.header.stamp.sec;
    time_stamp_.nanoseconds_ = ros_pose.header.stamp.nanosec;
    frame_id_ = ros_pose.header.frame_id;
    point_ = ros_helpers::fromRosPoint(ros_pose.pose.position);
    orientation_ = ros_helpers::fromRosQuaternion(ros_pose.pose.orientation);
  }

  Time time_stamp_;
  std::string frame_id_;
  Point point_;
  Quaternion orientation_;
};

struct RVIZ_COMMON_PUBLIC TransformStamped
{
  TransformStamped()
  : time_stamp_(), parent_frame_id_(""), child_frame_id_(""), translation_(), rotation_()
  {}
  explicit TransformStamped(geometry_msgs::msg::TransformStamped ros_transform)
  {
    time_stamp_.seconds_ = ros_transform.header.stamp.sec;
    time_stamp_.nanoseconds_ = ros_transform.header.stamp.nanosec;
    parent_frame_id_ = ros_transform.header.frame_id;
    child_frame_id_ = ros_transform.child_frame_id;
    translation_ = ros_helpers::fromRosVector3(ros_transform.transform.translation);
    rotation_ = ros_helpers::fromRosQuaternion(ros_transform.transform.rotation);
  }

  Time time_stamp_;
  std::string parent_frame_id_;
  std::string child_frame_id_;
  Point translation_;
  Quaternion rotation_;
};

}  // namespace transformation
}  // namespace rviz_common

#endif  // RVIZ_COMMON__TRANSFORMATION__STRUCTS_HPP_
