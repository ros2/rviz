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

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/header.hpp"

#include "rviz_common/transformation/structs.hpp"
#include "rviz_common/transformation/ros_helpers/ros_conversion_helpers.hpp"

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

rviz_common::transformation::Point createRvizCommonPoint(double x, double y, double z)
{
  rviz_common::transformation::Point point;
  point.x = x;
  point.y = y;
  point.z = z;

  return point;
}

rviz_common::transformation::Quaternion createRvizCommonQuaternion(
  double w, double x, double y, double z)
{
  rviz_common::transformation::Quaternion quaternion;
  quaternion.w = w;
  quaternion.x = x;
  quaternion.y = y;
  quaternion.z = z;

  return quaternion;
}

TEST(ros_conversion_helpers, fromRosPoint_converts_ros_point_to_rviz_common_transformation_point) {
  auto ros_point = createRosPoint(0, 1, 3);
  auto rviz_common_point = rviz_common::transformation::ros_helpers::fromRosPoint(ros_point);

  EXPECT_THAT(rviz_common_point, PointEq(ros_point));
}

TEST(
  ros_conversion_helpers,
  fromRosQuaternion_converts_ros_quaternion_to_rviz_common_transformation_quaternion) {
  auto ros_quaternion = createRosQuaternion(0.707, 0, 0.707, 0);
  auto rviz_common_quaternion =
    rviz_common::transformation::ros_helpers::fromRosQuaternion(ros_quaternion);

  EXPECT_THAT(rviz_common_quaternion, QuaternionEq(ros_quaternion));
}

TEST(
  ros_conversion_helpers, fromRosVector3_converts_ros_vector3_to_rviz_common_transformation_point) {
  auto ros_vector = createRosVector3(2, 1, 3);
  auto rviz_common_point = rviz_common::transformation::ros_helpers::fromRosVector3(ros_vector);

  EXPECT_THAT(rviz_common_point, PointEq(ros_vector));
}

TEST(ros_conversion_helpers, toRosPoint_converts_rviz_common_transformation_point_to_ros_point) {
  auto rviz_common_point = createRvizCommonPoint(4, 1, 3);
  auto ros_point = rviz_common::transformation::ros_helpers::toRosPoint(rviz_common_point);

  EXPECT_THAT(ros_point, PointEq(rviz_common_point));
}

TEST(
  ros_conversion_helpers, toRosVector3_converts_rviz_common_transformation_point_to_ros_vector3) {
  auto rviz_common_point = createRvizCommonPoint(4, 1, 0);
  auto ros_vector3 = rviz_common::transformation::ros_helpers::toRosVector3(rviz_common_point);

  EXPECT_THAT(ros_vector3, PointEq(rviz_common_point));
}

TEST(
  ros_conversion_helpers,
  toRosQuaternion_converts_rviz_common_transformation_quaternion_to_ros_quaternion) {
  auto rviz_common_quaternion = createRvizCommonQuaternion(0.707, 0, 0.707, 0);
  auto ros_quaternion =
    rviz_common::transformation::ros_helpers::toRosQuaternion(rviz_common_quaternion);

  EXPECT_THAT(ros_quaternion, QuaternionEq(rviz_common_quaternion));
}

TEST(ros_conversion_helpers, toRosHeader_creates_a_ros_header_from_time_and_frame_id) {
  rviz_common::transformation::Time rviz_common_time_stamp(2, 100);
  std::string frame_id = "test_frame";
  auto ros_header = rviz_common::transformation::ros_helpers::toRosHeader(
    rviz_common_time_stamp, frame_id);

  EXPECT_THAT(ros_header.stamp.sec, testing::Eq(2));
  EXPECT_THAT(ros_header.stamp.nanosec, testing::Eq(static_cast<uint32_t>(100)));
  EXPECT_THAT(ros_header.frame_id, testing::Eq(frame_id));
}

TEST(
  ros_conversion_helpers,
  toRosPoseStamped_creates_a_ros_pose_stamped_from_an_rviz_common_transformation_pose_stamped) {
  rviz_common::transformation::PoseStamped rviz_common_pose_stamped;
  rviz_common_pose_stamped.time_stamp_ = rviz_common::transformation::Time(2, 10);
  rviz_common_pose_stamped.frame_id_ = "pose_stamped_frame";
  rviz_common_pose_stamped.position_ = createRvizCommonPoint(1, 2, 3);
  rviz_common_pose_stamped.orientation_ = createRvizCommonQuaternion(0, 1, 0, 0);

  auto ros_pose_stamped = rviz_common::transformation::ros_helpers::toRosPoseStamped(
    rviz_common_pose_stamped);

  EXPECT_THAT(ros_pose_stamped.header.frame_id, testing::Eq(rviz_common_pose_stamped.frame_id_));
  EXPECT_THAT(ros_pose_stamped.pose.position, PointEq(rviz_common_pose_stamped.position_));
  EXPECT_THAT(
    ros_pose_stamped.header.stamp.sec, testing::Eq(rviz_common_pose_stamped.time_stamp_.seconds_));
  EXPECT_THAT(
    ros_pose_stamped.pose.orientation, QuaternionEq(rviz_common_pose_stamped.orientation_));
  EXPECT_THAT(
    ros_pose_stamped.header.stamp.nanosec,
    testing::Eq(rviz_common_pose_stamped.time_stamp_.nanoseconds_));
}

TEST(
  ros_conversion_helpers, fromRosPoseStamped_creates_an_rviz_common_pose_stamoed_from_a_ros_one) {
  std_msgs::msg::Header header;
  header.stamp.sec = 10;
  header.stamp.nanosec = 20;
  header.frame_id = "pose_frame";
  auto ros_pose = createRosPoseStamped(
    header, createRosPoint(3, 2, 1), createRosQuaternion(0, 0, 1, 0));
  auto rviz_common_pose_stamped =
    rviz_common::transformation::ros_helpers::fromRosPoseStamped(ros_pose);

  EXPECT_THAT(
    rviz_common_pose_stamped.time_stamp_.seconds_, testing::Eq(ros_pose.header.stamp.sec));
  EXPECT_THAT(
    rviz_common_pose_stamped.time_stamp_.nanoseconds_, testing::Eq(ros_pose.header.stamp.nanosec));
  EXPECT_THAT(rviz_common_pose_stamped.frame_id_, testing::Eq(ros_pose.header.frame_id));
  EXPECT_THAT(rviz_common_pose_stamped.position_, PointEq(ros_pose.pose.position));
  EXPECT_THAT(rviz_common_pose_stamped.orientation_, QuaternionEq(ros_pose.pose.orientation));
}

TEST(
  ros_conversion_helpers,
  fromRostransformStamped_creates_an_rviz_common_transform_stamoed_from_a_ros_one) {
  std_msgs::msg::Header header;
  header.stamp.sec = 10;
  header.stamp.nanosec = 20;
  header.frame_id = "parent_frame";
  auto ros_transform = createRosTransformStamped(
    header, "child_frame", createRosVector3(3, 2, 1), createRosQuaternion(0, 0, 1, 0));
  auto rviz_common_transform_stamped =
    rviz_common::transformation::ros_helpers::fromRosTransformStamped(ros_transform);

  EXPECT_THAT(
    rviz_common_transform_stamped.time_stamp_.nanoseconds_,
    testing::Eq(ros_transform.header.stamp.nanosec));
  EXPECT_THAT(
    rviz_common_transform_stamped.time_stamp_.seconds_,
    testing::Eq(ros_transform.header.stamp.sec));
  EXPECT_THAT(
    rviz_common_transform_stamped.child_frame_id_, testing::Eq(ros_transform.child_frame_id));
  EXPECT_THAT(
    rviz_common_transform_stamped.translation_, PointEq(ros_transform.transform.translation));
  EXPECT_THAT(
    rviz_common_transform_stamped.rotation_, QuaternionEq(ros_transform.transform.rotation));
  EXPECT_THAT(
    rviz_common_transform_stamped.parent_frame_id_, testing::Eq(ros_transform.header.frame_id));
}