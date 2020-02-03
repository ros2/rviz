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

#include <gmock/gmock.h>

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "rviz_default_plugins/transformation/tf_frame_transformer.hpp"

class FrameTransformerTfFixture : public testing::Test
{
public:
  geometry_msgs::msg::TransformStamped getTransformStamped(
    std::string frame = "frame", std::string fixed_frame = "fixed_frame")
  {
    geometry_msgs::msg::TransformStamped transform_stamped;
    std_msgs::msg::Header header;
    header.frame_id = fixed_frame;
    geometry_msgs::msg::Transform transform;
    transform.rotation = geometry_msgs::msg::Quaternion();
    transform.rotation.w = 1;
    transform.translation = geometry_msgs::msg::Vector3();

    transform_stamped.header = header;
    transform_stamped.child_frame_id = frame;
    transform_stamped.transform = transform;

    return transform_stamped;
  }

  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    auto node = std::make_shared<rclcpp::Node>("test_node");
    tf_wrapper_ = std::make_shared<rviz_default_plugins::transformation::TFWrapper>();
    tf_wrapper_->initializeBuffer(clock, node, false);
    tf_transformer_ = std::make_unique<rviz_default_plugins::transformation::TFFrameTransformer>(
      tf_wrapper_);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }

  geometry_msgs::msg::PoseStamped getPoseStamped()
  {
    std_msgs::msg::Header header;
    header.frame_id = "pose_frame";
    geometry_msgs::msg::Pose pose;
    pose.position = geometry_msgs::msg::Point();
    pose.orientation = geometry_msgs::msg::Quaternion();
    pose.orientation.w = 1;
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = header;
    pose_stamped.pose = pose;

    return pose_stamped;
  }

  std::shared_ptr<rviz_default_plugins::transformation::TFWrapper> tf_wrapper_;
  std::unique_ptr<rviz_default_plugins::transformation::TFFrameTransformer> tf_transformer_;
};

TEST_F(FrameTransformerTfFixture, frameHasProblems_returns_true_if_frame_does_not_exist) {
  std::string error;
  EXPECT_TRUE(tf_transformer_->frameHasProblems("any_frame", error));
}

TEST_F(FrameTransformerTfFixture, frameHasProblems_returns_false_if_frame_does_exist) {
  tf_wrapper_->getBuffer()->setTransform(getTransformStamped(), "test", true);

  std::string error;
  EXPECT_FALSE(tf_transformer_->frameHasProblems("fixed_frame", error));
  EXPECT_FALSE(tf_transformer_->frameHasProblems("frame", error));
}

TEST_F(FrameTransformerTfFixture, canTransform_returns_false_if_frame_does_not_exist) {
  tf_wrapper_->getBuffer()->setTransform(getTransformStamped(), "test", true);

  std::string error;
  std::string expected_error = "For frame [another_frame]: Frame [another_frame] does not exist";
  EXPECT_FALSE(
    tf_transformer_->canTransform(
      "fixed_frame", "another_frame", tf2::get_now(), &error));
  EXPECT_THAT(error, testing::StrEq(expected_error));
}

TEST_F(FrameTransformerTfFixture, canTransform_returns_false_if_fixed_frame_does_not_exist) {
  tf_wrapper_->getBuffer()->setTransform(getTransformStamped(), "test", true);

  std::string error;
  std::string expected_error =
    "For frame [frame]: Fixed Frame [another_fixed_frame] does not exist";
  EXPECT_FALSE(
    tf_transformer_->canTransform(
      "another_fixed_frame", "frame", tf2::get_now(), &error));
  EXPECT_THAT(error, testing::StrEq(expected_error));
}

TEST_F(FrameTransformerTfFixture, canTransform_returns_false_if_transform_does_not_exist) {
  tf_wrapper_->getBuffer()->setTransform(getTransformStamped(), "test", true);
  tf_wrapper_->getBuffer()->setTransform(
    getTransformStamped(
      "third_frame",
      "fourth_frame"), "test", true);

  std::string error;
  std::string partial_expected_error = "No transform to fixed frame";
  EXPECT_FALSE(tf_transformer_->canTransform("fourth_frame", "frame", tf2::get_now(), &error));
  EXPECT_THAT(error, testing::HasSubstr(partial_expected_error));
  EXPECT_THAT(error, testing::HasSubstr("fourth_frame"));
  EXPECT_THAT(error, testing::HasSubstr("frame"));
}

TEST_F(FrameTransformerTfFixture, canTransform_returns_true_if_transform_exists) {
  tf_wrapper_->getBuffer()->setTransform(getTransformStamped(), "test", true);

  std::string error;
  EXPECT_TRUE(
    tf_transformer_->canTransform(
      "fixed_frame", "frame", tf2::get_now(), &error));
}
