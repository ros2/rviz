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

#include <OgreEntity.h>
#include <OgreManualObject.h>
#include <OgreRoot.h>

#include "rviz_common/properties/enum_property.hpp"

#include "rviz_default_plugins/displays/odometry/odometry_display.hpp"
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

// *INDENT-OFF* - uncrustify cannot deal with layout of matrices here
std::array<double, 36> getCovariances3D()
{
  return std::array<double, 36>{{
    0.75, 0.04, 0.1,  0,    0,    0,
    0.04, 0.7,  0.4,  0,    0,    0,
    0.1,  0.4,  0.5,  0,    0,    0,
    0,    0,    0,    0.8,  0.25, 0.06,
    0,    0,    0,    0.25, 0.3,  0.22,
    0,    0,    0,    0.06, 0.22, 0.6
  }};
}
// *INDENT-ON*

  nav_msgs::msg::Odometry::SharedPtr createOdometryMessage()
  {
    auto odometry_msg = std::make_shared<nav_msgs::msg::Odometry>();
    odometry_msg->header = std_msgs::msg::Header();
    odometry_msg->header.stamp = rclcpp::Clock().now();
    odometry_msg->child_frame_id = "child_frame";

    odometry_msg->pose.pose.position.x = 1;
    odometry_msg->pose.pose.position.y = 2;
    odometry_msg->pose.pose.position.z = 3;
    odometry_msg->pose.pose.orientation.w = 0;
    odometry_msg->pose.pose.orientation.x = 0;
    odometry_msg->pose.pose.orientation.y = 1;
    odometry_msg->pose.pose.orientation.z = 0;
    odometry_msg->pose.covariance = getCovariances3D();

    odometry_msg->twist.twist.linear.x = 3;

    return odometry_msg;
  }

  Ogre::SceneNode * getParentAt(size_t n, Ogre::SceneNode * child_node)
  {
    auto parent_node = child_node;
    for (size_t i = 0; i < n; ++i) {
      parent_node = parent_node->getParentSceneNode();
    }
    return parent_node;
  }

  std::unique_ptr<rviz_default_plugins::displays::OdometryDisplay> display_;
};

TEST_F(OdometryDisplayFixture, processMessage_returns_early_if_message_has_invalid_floats) {
  // to be sure that the early return is due to the message and not to the missing transform
  mockValidTransform();
  auto invalid_pose_message = createOdometryMessage();
  invalid_pose_message->pose.pose.position.x = nan("NaN");

  display_->processMessage(invalid_pose_message);

  auto arrows = rviz_default_plugins::findAllArrows(scene_manager_->getRootSceneNode());
  auto axes = rviz_default_plugins::findAllAxes(scene_manager_->getRootSceneNode());
  EXPECT_THAT(arrows, IsEmpty());
  EXPECT_THAT(axes, IsEmpty());
}

TEST_F(OdometryDisplayFixture, processMessage_returns_early_if_message_has_invalid_quaternions) {
  // to be sure that the early return is due to the message and not to the missing transform
  mockValidTransform();
  auto invalid_orientation_message = createOdometryMessage();
  invalid_orientation_message->pose.pose.orientation.w = 33;

  display_->processMessage(invalid_orientation_message);

  auto arrows = rviz_default_plugins::findAllArrows(scene_manager_->getRootSceneNode());
  auto axes = rviz_default_plugins::findAllAxes(scene_manager_->getRootSceneNode());
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

TEST_F(
  OdometryDisplayFixture,
  processMessage_returns_early_if_message_position_and_orientation_are_close_to_previous_ones) {
  mockValidTransform();
  auto odometry_message = createOdometryMessage();
  display_->processMessage(odometry_message);
  auto arrows = rviz_default_plugins::findAllArrows(scene_manager_->getRootSceneNode());
  auto axes = rviz_default_plugins::findAllAxes(scene_manager_->getRootSceneNode());
  ASSERT_THAT(arrows, SizeIs(1));
  ASSERT_THAT(axes, SizeIs(1));
  arrows[0]->removeAllChildren();
  axes[0]->removeAllChildren();

  display_->processMessage(odometry_message);

  arrows = rviz_default_plugins::findAllArrows(scene_manager_->getRootSceneNode());
  axes = rviz_default_plugins::findAllAxes(scene_manager_->getRootSceneNode());
  EXPECT_THAT(arrows, IsEmpty());
  EXPECT_THAT(axes, IsEmpty());
}

TEST_F(OdometryDisplayFixture, processMessage_sets_arrow_and_axes_according_to_message) {
  mockValidTransform();
  auto effective_arrow_orientation =
    Ogre::Quaternion(0, 0, 1, 0) *
    Ogre::Quaternion(Ogre::Degree(-90), Ogre::Vector3::UNIT_Y) *
    Ogre::Quaternion(Ogre::Degree(-90), Ogre::Vector3::UNIT_X);
  auto odometry_message = createOdometryMessage();

  display_->processMessage(odometry_message);

  auto arrows = rviz_default_plugins::findAllArrows(scene_manager_->getRootSceneNode());
  auto axes = rviz_default_plugins::findAllAxes(scene_manager_->getRootSceneNode());
  ASSERT_THAT(arrows, SizeIs(1));
  ASSERT_THAT(axes, SizeIs(1));
  EXPECT_THAT(arrows[0]->getPosition(), Vector3Eq(Ogre::Vector3(0, 1, 0)));
  EXPECT_THAT(arrows[0]->getOrientation(), QuaternionEq(effective_arrow_orientation));
  EXPECT_THAT(axes[0]->getPosition(), Vector3Eq(Ogre::Vector3(0, 1, 0)));
  EXPECT_THAT(axes[0]->getOrientation(), QuaternionEq(Ogre::Quaternion(0, 0, 1, 0)));
}

TEST_F(
  OdometryDisplayFixture, processMessage_handles_arrow_and_axes_visibility_according_to_message) {
  mockValidTransform();
  auto odometry_message = createOdometryMessage();
  auto shape_property = static_cast<rviz_common::properties::EnumProperty *>(display_->childAt(4));
  ASSERT_THAT(shape_property->getNameStd(), StrEq("Shape"));

  display_->processMessage(odometry_message);

  EXPECT_TRUE(
    rviz_default_plugins::arrowIsVisible(
      rviz_default_plugins::findOneArrow(scene_manager_->getRootSceneNode())));
  EXPECT_FALSE(
    rviz_default_plugins::axesAreVisible(
      rviz_default_plugins::findOneAxes(scene_manager_->getRootSceneNode())));

  shape_property->setString("Axes");
  odometry_message->pose.pose.position.x = 35;
  display_->processMessage(odometry_message);

  EXPECT_FALSE(
    rviz_default_plugins::arrowIsVisible(
      rviz_default_plugins::findOneArrow(scene_manager_->getRootSceneNode())));
  EXPECT_TRUE(
    rviz_default_plugins::axesAreVisible(
      rviz_default_plugins::findOneAxes(scene_manager_->getRootSceneNode())));
}

TEST_F(OdometryDisplayFixture, processMessage_sets_covariance_visual_according_to_message) {
  mockValidTransform();
  auto odometry_message = createOdometryMessage();
  auto cov_position = Ogre::Vector3(0, 1, 0);
  auto cov_orientation = Ogre::Quaternion(0, 0, 1, 0);

  display_->processMessage(odometry_message);

  auto all_spheres = rviz_default_plugins::findAllSpheres(scene_manager_->getRootSceneNode());
  ASSERT_THAT(all_spheres, SizeIs(1));
  auto sphere_scene_node = all_spheres[0]->getParentSceneNode();
  auto cov_node_with_position_and_orientation = getParentAt(5, sphere_scene_node);
  EXPECT_THAT(cov_node_with_position_and_orientation->getPosition(), cov_position);
  EXPECT_THAT(cov_node_with_position_and_orientation->getOrientation(), cov_orientation);
  EXPECT_THAT(getParentAt(4, sphere_scene_node)->getOrientation(), cov_orientation.Inverse());
}

TEST_F(OdometryDisplayFixture, reset_clears_all_objects) {
  mockValidTransform();
  auto odometry_message = createOdometryMessage();

  display_->processMessage(odometry_message);
  display_->reset();

  auto all_spheres = rviz_default_plugins::findAllSpheres(scene_manager_->getRootSceneNode());
  auto all_cylynders = rviz_default_plugins::findAllCylinders(scene_manager_->getRootSceneNode());
  auto all_cones = rviz_default_plugins::findAllCones(scene_manager_->getRootSceneNode());
  EXPECT_THAT(all_spheres, SizeIs(0));
  EXPECT_THAT(all_cylynders, SizeIs(0));
  EXPECT_THAT(all_cones, SizeIs(0));
}
