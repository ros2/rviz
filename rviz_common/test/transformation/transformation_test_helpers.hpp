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

#ifndef TRANSFORMATION__TRANSFORMATION_TEST_HELPERS_HPP_
#define TRANSFORMATION__TRANSFORMATION_TEST_HELPERS_HPP_

#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/header.hpp"

MATCHER_P(PointEq, expected, "") {
  return expected.x == arg.x && expected.y == arg.y && expected.z == arg.z;
}

MATCHER_P(QuaternionEq, expected, "") {
  return expected.w == arg.w &&
         expected.x == arg.x &&
         expected.y == arg.y &&
         expected.z == arg.z;
}

geometry_msgs::msg::Point createRosPoint(double x, double y, double z)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = z;

  return point;
}

geometry_msgs::msg::Vector3 createRosVector3(double x, double y, double z)
{
  geometry_msgs::msg::Vector3 vector;
  vector.x = x;
  vector.y = y;
  vector.z = z;

  return vector;
}

geometry_msgs::msg::Quaternion createRosQuaternion(double w, double x, double y, double z)
{
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.w = w;
  quaternion.x = x;
  quaternion.y = y;
  quaternion.z = z;

  return quaternion;
}

std_msgs::msg::Header createRosHeader(int32_t sec, uint32_t nanosec, std::string frame)
{
  std_msgs::msg::Header header;
  header.stamp.sec = sec;
  header.stamp.nanosec = nanosec;
  header.frame_id = frame;

  return header;
}

geometry_msgs::msg::PoseStamped createRosPoseStamped(
  std_msgs::msg::Header header,
  geometry_msgs::msg::Point position,
  geometry_msgs::msg::Quaternion orientation)
{
  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header = header;
  pose_stamped.pose.position = position;
  pose_stamped.pose.orientation = orientation;

  return pose_stamped;
}

geometry_msgs::msg::TransformStamped createRosTransformStamped(
  std_msgs::msg::Header header,
  std::string child_frame_id,
  geometry_msgs::msg::Vector3 translation,
  geometry_msgs::msg::Quaternion rotation)
{
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header = header;
  transform_stamped.child_frame_id = child_frame_id;
  transform_stamped.transform.translation = translation;
  transform_stamped.transform.rotation = rotation;

  return transform_stamped;
}

#endif  // TRANSFORMATION__TRANSFORMATION_TEST_HELPERS_HPP_
