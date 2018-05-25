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
#include <vector>

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# pragma GCC diagnostic ignored "-Wpedantic"
#else
#pragma warning(push)
#pragma warning(disable : 4996)
#endif

#include <OgreRoot.h>
#include <OgreEntity.h>
#include <OgreManualObject.h>

#ifndef _WIN32
# pragma GCC diagnostic pop
#else
# pragma warning(pop)
#endif

#include "rviz_common/properties/enum_property.hpp"

#include "../../../../src/rviz_default_plugins/displays/odometry/odometry_display.hpp"
#include "../display_test_fixture.hpp"
#include "../../scene_graph_introspection.hpp"

using namespace ::testing;  // NOLINT

class OdometryDisplayFixture : public DisplayTestFixture
{
public:
  OdometryDisplayFixture()
  {
    display_ = std::make_unique<rviz_default_plugins::displays::OdometryDisplay>(
      context_.get(), scene_manager_->getRootSceneNode()->createChildSceneNode());
  }

  nav_msgs::msg::Odometry::SharedPtr createOdometryMessage()
  {
    auto odometry_msg = std::make_shared<nav_msgs::msg::Odometry>();
    odometry_msg->header = std_msgs::msg::Header();
    odometry_msg->header.stamp = rclcpp::Clock().now();
    odometry_msg->child_frame_id = "child_frame";

    odometry_msg->pose.pose.position.x = 1;
    odometry_msg->pose.pose.position.y = 2;
    odometry_msg->pose.pose.position.z = 3;
    odometry_msg->pose.pose.orientation.x = 1;
    odometry_msg->pose.pose.orientation.y = 0;
    odometry_msg->pose.pose.orientation.z = 0;
    odometry_msg->pose.pose.orientation.w = 0;

    odometry_msg->twist.twist.linear.x = 3;

    return odometry_msg;
  }

  std::unique_ptr<rviz_default_plugins::displays::OdometryDisplay> display_;
};

TEST_F(OdometryDisplayFixture, processMessage_returns_early_if_message_has_invalid_content) {
  // to be sure that the early return is due to the message and not to the missing transform
  mockValidTransform();

  auto invalid_pose_message = createOdometryMessage();
  invalid_pose_message->pose.pose.position.x = nan("NaN");
  display_->processMessage(invalid_pose_message);
  auto arrows = rviz_default_plugins::findAllArrows(scene_manager_->getRootSceneNode());
  auto axes = rviz_default_plugins::findAllAxes(scene_manager_->getRootSceneNode());
  EXPECT_THAT(arrows, IsEmpty());
  EXPECT_THAT(axes, IsEmpty());

  auto invalide_orientation_message = createOdometryMessage();
  invalide_orientation_message->pose.pose.orientation.w = 33;
  display_->processMessage(invalide_orientation_message);
  arrows = rviz_default_plugins::findAllArrows(scene_manager_->getRootSceneNode());
  axes = rviz_default_plugins::findAllAxes(scene_manager_->getRootSceneNode());
  EXPECT_THAT(arrows, IsEmpty());
  EXPECT_THAT(axes, IsEmpty());
}

TEST_F(OdometryDisplayFixture, processMessage_returns_early_if_transform_is_missing) {
  EXPECT_CALL(*frame_manager_, transform(_, _, _, _, _)).WillOnce(Return(false));  // NOLINT

  auto odometry_message = createOdometryMessage();
  display_->processMessage(odometry_message);
  auto arrows = rviz_default_plugins::findAllArrows(scene_manager_->getRootSceneNode());
  auto axes = rviz_default_plugins::findAllAxes(scene_manager_->getRootSceneNode());
  EXPECT_THAT(arrows, IsEmpty());
  EXPECT_THAT(axes, IsEmpty());
}

TEST_F(OdometryDisplayFixture,
  processMessage_returns_early_if_message_position_and_orientation_are_close_to_previous_ones) {
  mockValidTransform();

  auto odometry_message = createOdometryMessage();
  display_->processMessage(odometry_message);
  auto arrows = rviz_default_plugins::findAllArrows(scene_manager_->getRootSceneNode());
  auto axes = rviz_default_plugins::findAllAxes(scene_manager_->getRootSceneNode());
  ASSERT_THAT(arrows, SizeIs(1));
  ASSERT_THAT(axes, SizeIs(1));
  arrows[0]->removeAndDestroyAllChildren();
  axes[0]->removeAndDestroyAllChildren();
  display_->processMessage(odometry_message);

  auto new_arrows = rviz_default_plugins::findAllArrows(scene_manager_->getRootSceneNode());
  auto new_axes = rviz_default_plugins::findAllAxes(scene_manager_->getRootSceneNode());
  EXPECT_THAT(new_arrows, IsEmpty());
  EXPECT_THAT(new_axes, IsEmpty());
}

TEST_F(OdometryDisplayFixture, processMessage_sets_arrow_and_axes_correctly) {
  mockValidTransform();

  auto odometry_message = createOdometryMessage();
  display_->processMessage(odometry_message);
  auto arrows = rviz_default_plugins::findAllArrows(scene_manager_->getRootSceneNode());
  auto axes = rviz_default_plugins::findAllAxes(scene_manager_->getRootSceneNode());
  ASSERT_THAT(arrows, SizeIs(1));
  ASSERT_THAT(axes, SizeIs(1));

  auto effective_arrow_orientation =
    Ogre::Quaternion(0, 0, 1, 0) *
    Ogre::Quaternion(Ogre::Degree(-90), Ogre::Vector3::UNIT_Y) *
    Ogre::Quaternion(Ogre::Degree(-90), Ogre::Vector3::UNIT_X);
  EXPECT_THAT(arrows[0]->getPosition(), Vector3Eq(Ogre::Vector3(0, 1, 0)));
  EXPECT_THAT(arrows[0]->getOrientation(), QuaternionEq(effective_arrow_orientation));
  EXPECT_THAT(axes[0]->getPosition(), Vector3Eq(Ogre::Vector3(0, 1, 0)));
  EXPECT_THAT(axes[0]->getOrientation(), QuaternionEq(Ogre::Quaternion(0, 0, 1, 0)));
}

TEST_F(OdometryDisplayFixture, processMessage_handles_arrow_and_axes_visibility_correctly) {
  mockValidTransform();

  auto odometry_message = createOdometryMessage();
  display_->processMessage(odometry_message);
  auto shape_property = static_cast<rviz_common::properties::EnumProperty *>(display_->childAt(5));
  ASSERT_THAT(shape_property->getNameStd(), StrEq("Shape"));

  EXPECT_TRUE(rviz_default_plugins::arrowIsVisible(
      rviz_default_plugins::findOneArrow(scene_manager_->getRootSceneNode())));
  EXPECT_FALSE(rviz_default_plugins::axesAreVisible(
      rviz_default_plugins::findOneAxes(scene_manager_->getRootSceneNode())));

  shape_property->setString("Axes");
  odometry_message->pose.pose.position.x = 35;
  display_->processMessage(odometry_message);
  EXPECT_FALSE(rviz_default_plugins::arrowIsVisible(
      rviz_default_plugins::findOneArrow(scene_manager_->getRootSceneNode())));
  EXPECT_TRUE(rviz_default_plugins::axesAreVisible(
      rviz_default_plugins::findOneAxes(scene_manager_->getRootSceneNode())));
}
