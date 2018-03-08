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

#include <OgreRoot.h>
#include <OgreEntity.h>

#include "../../../../src/rviz_default_plugins/displays/pose_array/pose_array_display.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

#include "../display_test_fixture.hpp"
#include "../../scene_graph_introspection.hpp"

using namespace ::testing;  // NOLINT

class PoseArrayDisplayFixture : public DisplayTestFixture
{
public:
  void SetUp() override
  {
    DisplayTestFixture::SetUp();

    manual_object_ = scene_manager_->createManualObject();
    display_ = std::make_unique<rviz_default_plugins::displays::PoseArrayDisplay>(
      context_.get(),
      scene_manager_->getRootSceneNode()->createChildSceneNode(),
      manual_object_);
  }

  void TearDown() override
  {
    scene_manager_->destroyManualObject(manual_object_);
    display_.reset();
    DisplayTestFixture::TearDown();
  }

  void expectArrowIsVisible()
  {
    auto arrow_head = rviz_default_plugins::findEntityByMeshName(
      scene_manager_->getRootSceneNode(), "rviz_cone" ".mesh");
    auto arrow_shaft = rviz_default_plugins::findEntityByMeshName(
      scene_manager_->getRootSceneNode(), "rviz_cylinder.mesh");

    EXPECT_TRUE(arrow_head->isVisible() && arrow_shaft->isVisible());
  }

  void expectAxesAreVisible(Ogre::SceneNode * node)
  {
    for (uint16_t i = 0; i < 3; ++i) {
      auto child_node = dynamic_cast<Ogre::SceneNode *>(node->getChild(i)->getChild(0));
      auto entity = dynamic_cast<Ogre::Entity *>(child_node->getAttachedObject(0));
      ASSERT_TRUE(entity);
      EXPECT_TRUE(entity->isVisible());
    }
  }

  std::unique_ptr<rviz_default_plugins::displays::PoseArrayDisplay> display_;
  Ogre::ManualObject * manual_object_;
};

geometry_msgs::msg::PoseArray::ConstSharedPtr createMessageWithOnePose()
{
  auto message = std::make_shared<geometry_msgs::msg::PoseArray>();
  message->header = std_msgs::msg::Header();
  message->header.frame_id = "pose_array_frame";
  message->header.stamp = rclcpp::Clock().now();

  geometry_msgs::msg::Pose pose;
  pose.position.x = 1;
  pose.position.y = 2;
  pose.position.z = 3;
  pose.orientation.x = 1;
  pose.orientation.y = 0;
  pose.orientation.z = 1;
  pose.orientation.w = 0;

  message->poses.push_back(pose);

  return message;
}

TEST_F(PoseArrayDisplayFixture, setTransform_set_node_position_and_orientation_correctly) {
  mockValidTransform();
  auto msg = createMessageWithOnePose();
  display_->processMessage(msg);

  EXPECT_EQ(Ogre::Vector3(0, 1, 0), display_->getSceneNode()->getPosition());
  EXPECT_EQ(Ogre::Quaternion(0, 0, 1, 0), display_->getSceneNode()->getOrientation());
}

TEST_F(PoseArrayDisplayFixture, processMessage_sets_arrows3d_correctly) {
  mockValidTransform();
  auto msg = createMessageWithOnePose();

  display_->setShape("Arrow (3D)");
  display_->processMessage(msg);

  auto arrows = rviz_default_plugins::findAllArrows(scene_manager_->getRootSceneNode());

  size_t expected_number_of_arrows = 1;
  // The orientation is manipulated by the display and then in setOrientation() (see arrow.cpp)
  auto expected_orientation =
    Ogre::Quaternion(0, 1, 0, 1) *
    Ogre::Quaternion(Ogre::Degree(-90), Ogre::Vector3::UNIT_Y) *
    Ogre::Quaternion(Ogre::Degree(-90), Ogre::Vector3::UNIT_X);
  expected_orientation.normalise();

  EXPECT_EQ(expected_number_of_arrows, arrows.size());
  expectArrowIsVisible();
  EXPECT_VECTOR3_EQ(Ogre::Vector3(1, 2, 3), arrows[0]->getPosition());
  EXPECT_QUATERNION_EQ(expected_orientation, arrows[0]->getOrientation());
}

TEST_F(PoseArrayDisplayFixture, processMessage_sets_axes_correctly) {
  mockValidTransform();
  auto msg = createMessageWithOnePose();

  display_->setShape("Axes");
  display_->processMessage(msg);

  auto frames = rviz_default_plugins::findAllAxes(scene_manager_->getRootSceneNode());

  size_t expected_number_of_frames = 1;
  auto expected_orientation = Ogre::Quaternion(0, 1, 0, 1);
  expected_orientation.normalise();

  EXPECT_EQ(expected_number_of_frames, frames.size());
  expectAxesAreVisible(frames[0]);
  EXPECT_VECTOR3_EQ(Ogre::Vector3(1, 2, 3), frames[0]->getPosition());
  EXPECT_QUATERNION_EQ(expected_orientation, frames[0]->getOrientation());
}
