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

#include "rviz_common/transformation/ros_helpers/ros_conversion_helpers.hpp"

#include <string>

#include "rviz_common/transformation/structs.hpp"

namespace rviz_common
{
namespace transformation
{
namespace ros_helpers
{

rviz_common::transformation::Point fromRosPoint(geometry_msgs::msg::Point ros_point)
{
  Point point;
  point.x = ros_point.x;
  point.y = ros_point.y;
  point.z = ros_point.z;

  return point;
}

rviz_common::transformation::Quaternion fromRosQuaternion(
  geometry_msgs::msg::Quaternion ros_quaternion)
{
  Quaternion quaternion;
  quaternion.w = ros_quaternion.w;
  quaternion.x = ros_quaternion.x;
  quaternion.y = ros_quaternion.y;
  quaternion.z = ros_quaternion.z;

  return quaternion;
}

rviz_common::transformation::Point fromRosVector3(geometry_msgs::msg::Vector3 ros_vector)
{
  Point point;
  point.x = ros_vector.x;
  point.y = ros_vector.y;
  point.z = ros_vector.z;

  return point;
}

rviz_common::transformation::PoseStamped fromRosPoseStamped(
  geometry_msgs::msg::PoseStamped ros_pose)
{
  rviz_common::transformation::PoseStamped pose_stamped;
  pose_stamped.time_stamp.seconds = ros_pose.header.stamp.sec;
  pose_stamped.time_stamp.nanoseconds = ros_pose.header.stamp.nanosec;
  pose_stamped.frame_id = ros_pose.header.frame_id;
  pose_stamped.position = fromRosPoint(ros_pose.pose.position);
  pose_stamped.orientation = fromRosQuaternion(ros_pose.pose.orientation);

  return pose_stamped;
}

rviz_common::transformation::TransformStamped fromRosTransformStamped(
  geometry_msgs::msg::TransformStamped ros_transform)
{
  rviz_common::transformation::TransformStamped transform_stamped;
  transform_stamped.time_stamp.seconds = ros_transform.header.stamp.sec;
  transform_stamped.time_stamp.nanoseconds = ros_transform.header.stamp.nanosec;
  transform_stamped.parent_frame_id = ros_transform.header.frame_id;
  transform_stamped.child_frame_id = ros_transform.child_frame_id;
  transform_stamped.translation = fromRosVector3(ros_transform.transform.translation);
  transform_stamped.rotation = fromRosQuaternion(ros_transform.transform.rotation);

  return transform_stamped;
}

std_msgs::msg::Header toRosHeader(
  rviz_common::transformation::Time time_stamp, const std::string & frame_id)
{
  std_msgs::msg::Header ros_header;
  ros_header.stamp.sec = time_stamp.seconds;
  ros_header.stamp.nanosec = time_stamp.nanoseconds;
  ros_header.frame_id = frame_id;

  return ros_header;
}

geometry_msgs::msg::Point toRosPoint(rviz_common::transformation::Point point)
{
  geometry_msgs::msg::Point ros_point;
  ros_point.x = point.x;
  ros_point.y = point.y;
  ros_point.z = point.z;

  return ros_point;
}
geometry_msgs::msg::Vector3 toRosVector3(rviz_common::transformation::Point point)
{
  geometry_msgs::msg::Vector3 ros_vector;
  ros_vector.x = point.x;
  ros_vector.y = point.y;
  ros_vector.z = point.z;

  return ros_vector;
}


geometry_msgs::msg::Quaternion toRosQuaternion(
  rviz_common::transformation::Quaternion quaternion)
{
  geometry_msgs::msg::Quaternion ros_quaternion;
  ros_quaternion.w = quaternion.w;
  ros_quaternion.x = quaternion.x;
  ros_quaternion.y = quaternion.y;
  ros_quaternion.z = quaternion.z;

  return ros_quaternion;
}

geometry_msgs::msg::PoseStamped toRosPoseStamped(
  rviz_common::transformation::PoseStamped pose_stamped)
{
  geometry_msgs::msg::PoseStamped ros_pose;
  ros_pose.header = toRosHeader(pose_stamped.time_stamp, pose_stamped.frame_id);
  ros_pose.pose.position = toRosPoint(pose_stamped.position);
  ros_pose.pose.orientation = toRosQuaternion(pose_stamped.orientation);

  return ros_pose;
}

}  // namespace ros_helpers
}  // namespace transformation
}  // namespace rviz_common
