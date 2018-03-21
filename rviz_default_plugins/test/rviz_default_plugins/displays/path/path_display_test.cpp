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


#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <memory>
#include <string>
#include <vector>

#ifdef _WIN32
#pragma warning(push)
#pragma warning(disable : 4996)
#include <OgreEntity.h>
#pragma warning(pop)
#else
#include <OgreEntity.h>
#endif
#include <OgreMesh.h>
#include <OgreManualObject.h>

#include "visualization_msgs/msg/marker.hpp"
#include "rviz_rendering/objects/arrow.hpp"
#include "rviz_rendering/objects/shape.hpp"
#include "../../scene_graph_introspection.hpp"

#include "../../../../src/rviz_default_plugins/displays/path/path_display.hpp"

#include "../display_test_fixture.hpp"

using namespace ::testing;  // NOLINT

class PathTestFixture : public DisplayTestFixture
{
public:
  void SetUp() override
  {
    DisplayTestFixture::SetUp();
    path_display_ = std::make_shared<rviz_default_plugins::displays::PathDisplay>(context_.get());
  }

  void TearDown() override
  {
    path_display_.reset();
    DisplayTestFixture::TearDown();
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

void assertArrowIsNotVisible()
{
  auto arrow_head = rviz_default_plugins::findEntityByMeshName(
    PathTestFixture::scene_manager_->getRootSceneNode(), "rviz_cone" ".mesh");
  auto arrow_shaft = rviz_default_plugins::findEntityByMeshName(
    PathTestFixture::scene_manager_->getRootSceneNode(), "rviz_cylinder.mesh");

  ASSERT_TRUE(!arrow_head->isVisible() && !arrow_shaft->isVisible());
}

void assertArrowWithTransform(
  Ogre::SceneManager * scene_manager,
  Ogre::Vector3 position,
  Ogre::Vector3 scale,
  Ogre::Quaternion orientation)
{
  auto arrow_scene_node = rviz_default_plugins::findOneArrow(scene_manager->getRootSceneNode());
  ASSERT_TRUE(arrow_scene_node);
  EXPECT_VECTOR3_EQ(position, arrow_scene_node->getPosition());
  // Have to mangle the scale because of the default orientation of the cylinders (see arrow.cpp).
  EXPECT_VECTOR3_EQ(Ogre::Vector3(scale.z, scale.x, scale.y), arrow_scene_node->getScale());
  EXPECT_QUATERNION_EQ(orientation, arrow_scene_node->getOrientation());
}

TEST_F(PathTestFixture, processMessage_adds_nothing_to_scene_if_invalid_transformation) {
  EXPECT_CALL(*frame_manager_, getTransform(_, _, _, _)).WillOnce(Return(false));  // NOLINT

  path_display_->processMessage(createPathMessage());

  auto object = rviz_default_plugins::findOneManualObject(scene_manager_->getRootSceneNode());
  EXPECT_EQ(0u, object->getNumSections());
}

TEST_F(PathTestFixture, processMessage_adds_vertices_to_scene) {
  auto position = Ogre::Vector3::ZERO;
  auto orientation = Ogre::Quaternion::IDENTITY;
  mockValidTransform(position, orientation);

  path_display_->processMessage(createPathMessage());

  auto object = rviz_default_plugins::findOneManualObject(scene_manager_->getRootSceneNode());
  EXPECT_EQ(2u, object->getSection(0)->getRenderOperation()->vertexData->vertexCount);
}

TEST_F(PathTestFixture, processMessage_transforms_the_vertices_correctly) {
  auto position = Ogre::Vector3(1, 2, 3);
  auto orientation = Ogre::Quaternion::IDENTITY;
  mockValidTransform(position, orientation);

  path_display_->processMessage(createPathMessage());

  auto object = rviz_default_plugins::findOneManualObject(scene_manager_->getRootSceneNode());
  EXPECT_EQ(2u, object->getSection(0)->getRenderOperation()->vertexData->vertexCount);

  // Use bounding box to indirectly assert the vertices
  EXPECT_VECTOR3_EQ(Ogre::Vector3(2, 3, 3), object->getBoundingBox().getMinimum());
  EXPECT_VECTOR3_EQ(Ogre::Vector3(5, 4, 4), object->getBoundingBox().getMaximum());
}

TEST_F(PathTestFixture, processMessage_adds_billboard_line_to_scene) {
  ASSERT_EQ("Line Style", path_display_->childAt(2)->getNameStd());
  path_display_->childAt(2)->setValue("Billboards");

  auto position = Ogre::Vector3::ZERO;
  auto orientation = Ogre::Quaternion::IDENTITY;
  mockValidTransform(position, orientation);

  path_display_->processMessage(createPathMessage());

  auto object = rviz_default_plugins::findOneBillboardChain(scene_manager_->getRootSceneNode());
  EXPECT_EQ(1u, object->getNumberOfChains());
  EXPECT_EQ(2u, object->getNumChainElements(0));

  EXPECT_VECTOR3_EQ(Ogre::Vector3(4, 2, 0), object->getChainElement(0, 0).position);
  EXPECT_VECTOR3_EQ(Ogre::Vector3(1, 1, 1), object->getChainElement(0, 1).position);
}

TEST_F(PathTestFixture, processMessage_adds_axes_to_scene) {
  ASSERT_EQ("Pose Style", path_display_->childAt(8)->getNameStd());
  path_display_->childAt(8)->setValue("Axes");

  auto position = Ogre::Vector3::ZERO;
  auto orientation = Ogre::Quaternion::IDENTITY;
  mockValidTransform(position, orientation);

  path_display_->processMessage(createPathMessage());

  auto axes = rviz_default_plugins::findAllAxes(scene_manager_->getRootSceneNode());
  EXPECT_EQ(2u, axes.size());

  auto axes_positions = rviz_default_plugins::getPositionsFromNodes(axes);
  EXPECT_THAT(axes_positions, Contains(EqVector3(Ogre::Vector3(4, 2, 0))));
  EXPECT_THAT(axes_positions, Contains(EqVector3(Ogre::Vector3(1, 1, 1))));
}

TEST_F(PathTestFixture, processMessage_adds_arrows_to_scene) {
  ASSERT_EQ("Pose Style", path_display_->childAt(8)->getNameStd());
  path_display_->childAt(8)->setValue("Arrows");

  auto position = Ogre::Vector3::ZERO;
  auto orientation = Ogre::Quaternion::IDENTITY;
  mockValidTransform(position, orientation);

  path_display_->processMessage(createPathMessage());

  auto arrows = rviz_default_plugins::findAllArrows(scene_manager_->getRootSceneNode());
  EXPECT_THAT(arrows.size(), Eq(2u));

  auto arrow_positions = rviz_default_plugins::getPositionsFromNodes(arrows);
  EXPECT_THAT(arrow_positions, Contains(EqVector3(Ogre::Vector3(1, 1, 1))));
  EXPECT_THAT(arrow_positions, Contains(EqVector3(Ogre::Vector3(4, 2, 0))));

  // default orientation is set to (0.5, -0.5, -0.5, -0.5) by arrow
  auto default_orientation = Ogre::Quaternion(0.5f, -0.5f, -0.5f, -0.5f);
  auto arrow_orientations = rviz_default_plugins::getOrientationsFromNodes(arrows);
  EXPECT_THAT(arrow_orientations, Each(EqQuaternion(default_orientation)));
}
