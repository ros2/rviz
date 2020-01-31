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
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "../src/rviz_common/frame_manager.hpp"
#include "mock_frame_transformer.hpp"

using namespace ::testing;  // NOLINT

class FrameManagerTestFixture : public testing::Test
{
public:
  FrameManagerTestFixture()
  {
    clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    frame_transformer_ = std::make_shared<MockFrameTransformer>();
    frame_manager_ = std::make_unique<rviz_common::FrameManager>(clock_, frame_transformer_);
  }

  std::unique_ptr<rviz_common::FrameManager> frame_manager_;
  std::shared_ptr<MockFrameTransformer> frame_transformer_;
  std::shared_ptr<rclcpp::Clock> clock_;
};


TEST_F(FrameManagerTestFixture, transform_uses_transform_of_transformer) {
  geometry_msgs::msg::PoseStamped dummy_pose;
  EXPECT_CALL(*frame_transformer_, transform(_, _)).WillOnce(Return(dummy_pose));

  geometry_msgs::msg::Pose pose;
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  frame_manager_->transform(
    "any_frame", rclcpp::Time(0, 0, clock_->get_clock_type()), pose, position, orientation);
}

TEST_F(FrameManagerTestFixture, lastAvailableTransform_used_when_using_sync_approx) {
  geometry_msgs::msg::PoseStamped dummy_pose;
  EXPECT_CALL(*frame_transformer_, canTransform(_, _, _, _)).WillOnce(Return(true));
  EXPECT_CALL(*frame_transformer_, transform(_, _)).WillOnce(Return(dummy_pose));  // NOLINT

  frame_manager_->setSyncMode(rviz_common::FrameManager::SyncApprox);
  geometry_msgs::msg::Pose pose;
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  frame_manager_->transform(
    "any_frame", rclcpp::Time(0, 0, clock_->get_clock_type()), pose, position, orientation);
}

TEST_F(FrameManagerTestFixture, getAllFrameNames_uses_transformer_method) {
  std::vector<std::string> frame_names = {"test_frame"};
  EXPECT_CALL(*frame_transformer_, getAllFrameNames()).WillOnce(Return(frame_names));

  auto frames = frame_manager_->getAllFrameNames();
  ASSERT_THAT(frames, SizeIs(1));
  EXPECT_THAT(frames[0], Eq(frame_names[0]));
}

TEST_F(FrameManagerTestFixture, transformHasProblems_uses_transformer_method) {
  std::string source_frame_name = "test_frame";
  EXPECT_CALL(
    *frame_transformer_, canTransform(_, source_frame_name, _, _)).WillOnce(Return(true));

  std::string error;
  EXPECT_FALSE(
    frame_manager_->transformHasProblems(
      source_frame_name, rclcpp::Time(0, 0, clock_->get_clock_type()), error));
}

TEST_F(FrameManagerTestFixture, frameHasProblems_uses_transformer_method) {
  std::string frame_name = "test_frame";
  EXPECT_CALL(*frame_transformer_, frameHasProblems(frame_name, _)).WillOnce(Return(true));

  std::string error;
  EXPECT_TRUE(frame_manager_->frameHasProblems(frame_name, error));
}
