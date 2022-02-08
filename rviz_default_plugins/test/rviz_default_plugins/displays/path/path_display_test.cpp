/*
 * Copyright (c) 2018, Bosch Software Innovations GmbH.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
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

#include <OgreEntity.h>
#include <OgreMesh.h>
#include <OgreManualObject.h>

#include "visualization_msgs/msg/marker.hpp"
#include "rviz_rendering/objects/arrow.hpp"
#include "rviz_rendering/objects/shape.hpp"
#include "../../scene_graph_introspection.hpp"

#include "rviz_default_plugins/displays/path/path_display.hpp"

#include "../display_test_fixture.hpp"

using namespace ::testing;  // NOLINT

class PathTestFixture : public DisplayTestFixture
{
public:
  PathTestFixture()
  {
    path_display_ = std::make_shared<rviz_default_plugins::displays::PathDisplay>(context_.get());
  }

  std::shared_ptr<rviz_default_plugins::displays::PathDisplay> path_display_;
};

nav_msgs::msg::Path::ConstSharedPtr createPathMessage()
{
  auto message = std::make_shared<nav_msgs::msg::Path>();

  message->header = std_msgs::msg::Header();
  message->header.frame_id = "path_frame";
  message->header.stamp = rclcpp::Clock().now();

  auto pose = geometry_msgs::msg::PoseStamped();
  pose.pose.position.x = 1;
  pose.pose.position.y = 1;
  pose.pose.position.z = 1;

  auto orientation = Ogre::Quaternion::IDENTITY;
  pose.pose.orientation.w = orientation.w;
  pose.pose.orientation.x = orientation.x;
  pose.pose.orientation.y = orientation.y;
  pose.pose.orientation.z = orientation.z;

  auto pose2 = geometry_msgs::msg::PoseStamped();
  pose2.pose.position.x = 4;
  pose2.pose.position.y = 2;
  pose2.pose.position.z = 0;

  auto orientation2 = Ogre::Quaternion::IDENTITY;
  pose2.pose.orientation.w = orientation2.w;
  pose2.pose.orientation.x = orientation2.x;
  pose2.pose.orientation.y = orientation2.y;
  pose2.pose.orientation.z = orientation2.z;

  message->poses = std::vector<geometry_msgs::msg::PoseStamped>({pose, pose2});

  return message;
}

TEST_F(PathTestFixture, processMessage_adds_nothing_to_scene_if_invalid_transformation) {
  EXPECT_CALL(*frame_manager_, getTransform(_, _, _, _)).WillOnce(Return(false));  // NOLINT

  path_display_->processMessage(createPathMessage());

  auto object = rviz_default_plugins::findOneManualObject(scene_manager_->getRootSceneNode());
  EXPECT_THAT(object->getSections().size(), Eq(0u));
}

TEST_F(PathTestFixture, processMessage_adds_vertices_to_scene) {
  auto position = Ogre::Vector3::ZERO;
  auto orientation = Ogre::Quaternion::IDENTITY;
  mockValidTransform(position, orientation);

  path_display_->processMessage(createPathMessage());

  auto object = rviz_default_plugins::findOneManualObject(scene_manager_->getRootSceneNode());
  EXPECT_THAT(object->getSections()[0]->getRenderOperation()->vertexData->vertexCount, Eq(2u));
}

TEST_F(PathTestFixture, reset_clears_the_scene) {
  auto position = Ogre::Vector3::ZERO;
  auto orientation = Ogre::Quaternion::IDENTITY;
  mockValidTransform(position, orientation);

  path_display_->processMessage(createPathMessage());
  path_display_->reset();

  auto object = rviz_default_plugins::findOneManualObject(scene_manager_->getRootSceneNode());
  EXPECT_THAT(object->getSections().size(), Eq(0u));
}

TEST_F(PathTestFixture, reset_is_idempotent) {
  auto position = Ogre::Vector3::ZERO;
  auto orientation = Ogre::Quaternion::IDENTITY;
  mockValidTransform(position, orientation);
  path_display_->processMessage(createPathMessage());

  path_display_->reset();
  path_display_->reset();

  ASSERT_TRUE(1);
}

TEST_F(PathTestFixture, reset_removes_all_axes) {
  path_display_->findProperty("Pose Style")->setValue("Axes");

  auto position = Ogre::Vector3::ZERO;
  auto orientation = Ogre::Quaternion::IDENTITY;
  mockValidTransform(position, orientation);

  path_display_->processMessage(createPathMessage());
  EXPECT_THAT(rviz_default_plugins::findAllAxes(scene_manager_->getRootSceneNode()), SizeIs(2));

  path_display_->reset();
  EXPECT_THAT(rviz_default_plugins::findAllAxes(scene_manager_->getRootSceneNode()), SizeIs(0));
}

TEST_F(PathTestFixture, reset_removes_all_arrows) {
  path_display_->findProperty("Pose Style")->setValue("Arrows");

  auto position = Ogre::Vector3::ZERO;
  auto orientation = Ogre::Quaternion::IDENTITY;
  mockValidTransform(position, orientation);

  path_display_->processMessage(createPathMessage());
  EXPECT_THAT(rviz_default_plugins::findAllArrows(scene_manager_->getRootSceneNode()), SizeIs(2));

  path_display_->reset();
  EXPECT_THAT(rviz_default_plugins::findAllArrows(scene_manager_->getRootSceneNode()), SizeIs(0));
}

TEST_F(PathTestFixture, processMessage_transforms_the_vertices_correctly) {
  auto position = Ogre::Vector3(1, 2, 3);
  auto orientation = Ogre::Quaternion::IDENTITY;
  mockValidTransform(position, orientation);

  path_display_->processMessage(createPathMessage());

  auto object = rviz_default_plugins::findOneManualObject(scene_manager_->getRootSceneNode());
  EXPECT_THAT(object->getSections()[0]->getRenderOperation()->vertexData->vertexCount, Eq(2u));

  // Use bounding box to indirectly assert the vertices
  EXPECT_THAT(object->getBoundingBox().getMinimum(), Vector3Eq(Ogre::Vector3(2, 3, 3)));
  EXPECT_THAT(object->getBoundingBox().getMaximum(), Vector3Eq(Ogre::Vector3(5, 4, 4)));
}

TEST_F(PathTestFixture, processMessage_adds_billboard_line_to_scene) {
  path_display_->findProperty("Line Style")->setValue("Billboards");

  auto position = Ogre::Vector3::ZERO;
  auto orientation = Ogre::Quaternion::IDENTITY;
  mockValidTransform(position, orientation);

  path_display_->processMessage(createPathMessage());

  auto object = rviz_default_plugins::findOneBillboardChain(scene_manager_->getRootSceneNode());
  EXPECT_THAT(object->getNumberOfChains(), Eq(1u));
  EXPECT_THAT(object->getNumChainElements(0), Eq(2u));

  EXPECT_THAT(object->getChainElement(0, 0).position, Vector3Eq(Ogre::Vector3(4, 2, 0)));
  EXPECT_THAT(object->getChainElement(0, 1).position, Vector3Eq(Ogre::Vector3(1, 1, 1)));
}

TEST_F(PathTestFixture, processMessage_adds_axes_to_scene) {
  path_display_->findProperty("Pose Style")->setValue("Axes");

  auto position = Ogre::Vector3::ZERO;
  auto orientation = Ogre::Quaternion::IDENTITY;
  mockValidTransform(position, orientation);

  path_display_->processMessage(createPathMessage());

  auto axes = rviz_default_plugins::findAllAxes(scene_manager_->getRootSceneNode());
  EXPECT_THAT(axes, SizeIs(2));

  auto axes_positions = rviz_default_plugins::getPositionsFromNodes(axes);
  EXPECT_THAT(axes_positions, Contains(Vector3Eq(Ogre::Vector3(4, 2, 0))));
  EXPECT_THAT(axes_positions, Contains(Vector3Eq(Ogre::Vector3(1, 1, 1))));
}

TEST_F(PathTestFixture, processMessage_adds_arrows_to_scene) {
  path_display_->findProperty("Pose Style")->setValue("Arrows");

  auto position = Ogre::Vector3::ZERO;
  auto orientation = Ogre::Quaternion::IDENTITY;
  mockValidTransform(position, orientation);

  path_display_->processMessage(createPathMessage());

  auto arrows = rviz_default_plugins::findAllArrows(scene_manager_->getRootSceneNode());
  EXPECT_THAT(arrows, SizeIs(2));

  auto arrow_positions = rviz_default_plugins::getPositionsFromNodes(arrows);
  EXPECT_THAT(arrow_positions, Contains(Vector3Eq(Ogre::Vector3(1, 1, 1))));
  EXPECT_THAT(arrow_positions, Contains(Vector3Eq(Ogre::Vector3(4, 2, 0))));

  // default orientation is set to (0.5, -0.5, -0.5, -0.5) by arrow
  auto default_orientation = Ogre::Quaternion(0.5f, -0.5f, -0.5f, -0.5f);
  auto arrow_orientations = rviz_default_plugins::getOrientationsFromNodes(arrows);
  EXPECT_THAT(arrow_orientations, Each(QuaternionEq(default_orientation)));
}
