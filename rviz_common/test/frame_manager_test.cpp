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

#include "rclcpp/rclcpp.hpp"

#include "../src/rviz_common/frame_manager.hpp"
#include "mock_frame_transformer.hpp"

using namespace ::testing;  // NOLINT


TEST(FrameManager, transform_uses_transform_of_transformer) {
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  auto frame_transformer = std::make_shared<MockFrameTransformer>();
  rviz_common::transformation::PoseStamped dummy_pose;
  EXPECT_CALL(*frame_transformer, transform(_, _)).WillOnce(Return(dummy_pose));
  auto frame_manager = std::make_shared<rviz_common::FrameManager>(clock, frame_transformer);

  geometry_msgs::msg::Pose pose;
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  frame_manager->transform(
    "any_frame", rclcpp::Time(0, 0, clock->get_clock_type()), pose, position, orientation);
}

TEST(FrameManager, lastAvailableTransform_used_when_using_sync_approx) {
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  auto frame_transformer = std::make_shared<MockFrameTransformer>();
  rviz_common::transformation::PoseStamped dummy_pose;
  EXPECT_CALL(*frame_transformer, transformIsAvailable(_, _)).WillOnce(Return(true));
  EXPECT_CALL(*frame_transformer, transform(_, _)).WillOnce(Return(dummy_pose));  // NOLINT
  auto frame_manager = std::make_shared<rviz_common::FrameManager>(clock, frame_transformer);

  frame_manager->setSyncMode(rviz_common::FrameManager::SyncApprox);
  geometry_msgs::msg::Pose pose;
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  frame_manager->transform(
    "any_frame", rclcpp::Time(0, 0, clock->get_clock_type()), pose, position, orientation);
}
