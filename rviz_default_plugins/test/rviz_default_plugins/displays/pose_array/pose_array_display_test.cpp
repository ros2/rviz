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
#include <vector>

#include <OgreRoot.h>
#include <OgreEntity.h>
#include <OgreManualObject.h>

#include "rviz_common/properties/float_property.hpp"

#include "../../../../src/rviz_default_plugins/displays/pose_array/pose_array_display.hpp"

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

    arrow_2d_length_property_ = display_->childAt(5);

    for (int i = 3; i < 5; i++) {
      common_arrow_properties_.push_back(display_->childAt(i));
    }
    for (int i = 6; i < 10; i++) {
      arrow_3d_properties_.push_back(display_->childAt(i));
    }
    for (int i = 10; i < 12; i++) {
      axes_properties_.push_back(display_->childAt(i));
    }
  }

  void TearDown() override
  {
    scene_manager_->destroyManualObject(manual_object_);
    display_.reset();
    DisplayTestFixture::TearDown();
  }

  std::unique_ptr<rviz_default_plugins::displays::PoseArrayDisplay> display_;
  Ogre::ManualObject * manual_object_;

  rviz_common::properties::Property * arrow_2d_length_property_;
  std::vector<rviz_common::properties::Property *> common_arrow_properties_;
  std::vector<rviz_common::properties::Property *> arrow_3d_properties_;
  std::vector<rviz_common::properties::Property *> axes_properties_;
};

geometry_msgs::msg::PoseArray::SharedPtr createMessageWithOnePose()
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

void expectAxesAreVisible(Ogre::SceneNode * node)
{
  for (uint16_t i = 0; i < 3; ++i) {
    auto child_node = dynamic_cast<Ogre::SceneNode *>(node->getChild(i)->getChild(0));
    auto entity = dynamic_cast<Ogre::Entity *>(child_node->getAttachedObject(0));
    ASSERT_TRUE(entity);
    EXPECT_TRUE(entity->isVisible());
  }
}

TEST_F(PoseArrayDisplayFixture, constructor_set_all_the_properties_in_the_right_order) {
  EXPECT_EQ("Color", display_->childAt(3)->getNameStd());
  EXPECT_EQ("Alpha", display_->childAt(4)->getNameStd());
  EXPECT_EQ("Arrow Length", display_->childAt(5)->getNameStd());
  EXPECT_EQ("Head Radius", display_->childAt(6)->getNameStd());
  EXPECT_EQ("Head Length", display_->childAt(7)->getNameStd());
  EXPECT_EQ("Shaft Radius", display_->childAt(8)->getNameStd());
  EXPECT_EQ("Shaft Length", display_->childAt(9)->getNameStd());
  EXPECT_EQ("Axes Length", display_->childAt(10)->getNameStd());
  EXPECT_EQ("Axes Radius", display_->childAt(11)->getNameStd());
}

TEST_F(PoseArrayDisplayFixture, at_startup_only_flat_arrows_propertie_are_visible) {
  for (const auto & property : common_arrow_properties_) {
    EXPECT_FALSE(property->getHidden());
  }
  for (const auto & property : arrow_3d_properties_) {
    EXPECT_TRUE(property->getHidden());
  }
  for (const auto & property : axes_properties_) {
    EXPECT_TRUE(property->getHidden());
  }
  EXPECT_FALSE(arrow_2d_length_property_->getHidden());
}

TEST_F(PoseArrayDisplayFixture,
  processMessage_corrctly_manages_property_visibility_from_arrow2d_to_arrow3d) {
  mockValidTransform();
  auto msg = createMessageWithOnePose();
  display_->setShape("Arrow (3D)");
  display_->processMessage(msg);

  for (const auto & property : common_arrow_properties_) {
    EXPECT_FALSE(property->getHidden());
  }
  for (const auto & property : arrow_3d_properties_) {
    EXPECT_FALSE(property->getHidden());
  }
  for (const auto & property : axes_properties_) {
    EXPECT_TRUE(property->getHidden());
  }
  EXPECT_TRUE(arrow_2d_length_property_->getHidden());
}

TEST_F(PoseArrayDisplayFixture,
  processMessage_corrctly_manages_property_visibility_from_arrow2d_to_axes) {
  mockValidTransform();
  auto msg = createMessageWithOnePose();
  display_->setShape("Axes");
  display_->processMessage(msg);

  for (const auto & property : common_arrow_properties_) {
    EXPECT_TRUE(property->getHidden());
  }
  for (const auto & property : arrow_3d_properties_) {
    EXPECT_TRUE(property->getHidden());
  }
  for (const auto & property : axes_properties_) {
    EXPECT_FALSE(property->getHidden());
  }
  EXPECT_TRUE(arrow_2d_length_property_->getHidden());
}

TEST_F(PoseArrayDisplayFixture, setTransform_with_invalid_message_returns_early) {
  mockValidTransform();
  auto msg = createMessageWithOnePose();
  msg->poses[0].position.x = nan("NaN");
  display_->processMessage(msg);

  auto arrows_3d = rviz_default_plugins::findAllArrows(scene_manager_->getRootSceneNode());
  auto axes = rviz_default_plugins::findAllAxes(scene_manager_->getRootSceneNode());

  // the default position and orientation of the scene node are (0, 0, 0) and (1, 0, 0, 0)
  EXPECT_VECTOR3_EQ(Ogre::Vector3(0, 0, 0), display_->getSceneNode()->getPosition());
  EXPECT_QUATERNION_EQ(Ogre::Quaternion(1, 0, 0, 0), display_->getSceneNode()->getOrientation());
  EXPECT_FLOAT_EQ(0, manual_object_->getBoundingRadius());
  EXPECT_THAT(arrows_3d, SizeIs(0));
  EXPECT_THAT(axes, SizeIs(0));
}

TEST_F(PoseArrayDisplayFixture, setTransform_with_invalid_transform_returns_early) {
  EXPECT_CALL(*frame_manager_, getTransform(_, _, _, _)).WillOnce(Return(false));

  auto msg = createMessageWithOnePose();
  display_->processMessage(msg);

  auto arrows_3d = rviz_default_plugins::findAllArrows(scene_manager_->getRootSceneNode());
  auto axes = rviz_default_plugins::findAllAxes(scene_manager_->getRootSceneNode());

  // the default position and orientation of the scene node are (0, 0, 0) and (1, 0, 0, 0)
  EXPECT_VECTOR3_EQ(Ogre::Vector3(0, 0, 0), display_->getSceneNode()->getPosition());
  EXPECT_QUATERNION_EQ(Ogre::Quaternion(1, 0, 0, 0), display_->getSceneNode()->getOrientation());
  EXPECT_FLOAT_EQ(0, manual_object_->getBoundingRadius());
  EXPECT_THAT(arrows_3d, SizeIs(0));
  EXPECT_THAT(axes, SizeIs(0));
}

TEST_F(PoseArrayDisplayFixture, setTransform_sets_node_position_and_orientation_correctly) {
  mockValidTransform();
  auto msg = createMessageWithOnePose();
  display_->processMessage(msg);

  EXPECT_VECTOR3_EQ(Ogre::Vector3(0, 1, 0), display_->getSceneNode()->getPosition());
  EXPECT_QUATERNION_EQ(Ogre::Quaternion(0, 0, 1, 0), display_->getSceneNode()->getOrientation());
}

TEST_F(PoseArrayDisplayFixture, processMessage_sets_manualObject_correctly) {
  mockValidTransform();
  auto msg = createMessageWithOnePose();
  display_->processMessage(msg);

  auto manual_object_bounding_radius = 4.17732;
  EXPECT_FLOAT_EQ(manual_object_bounding_radius, manual_object_->getBoundingRadius());
  EXPECT_VECTOR3_EQ(Ogre::Vector3(0.85, 2, 3.3), manual_object_->getBoundingBox().getCenter());
}

TEST_F(PoseArrayDisplayFixture, processMessage_sets_arrows3d_correctly) {
  mockValidTransform();
  auto msg = createMessageWithOnePose();

  display_->setShape("Arrow (3D)");
  display_->processMessage(msg);

  auto arrows = rviz_default_plugins::findAllArrows(scene_manager_->getRootSceneNode());

  // The orientation is first manipulated by the display and then in setOrientation() in arrow.cpp
  auto expected_orientation =
    Ogre::Quaternion(0, 1, 0, 1) *
    Ogre::Quaternion(Ogre::Degree(-90), Ogre::Vector3::UNIT_Y) *
    Ogre::Quaternion(Ogre::Degree(-90), Ogre::Vector3::UNIT_X);
  expected_orientation.normalise();

  EXPECT_TRUE(rviz_default_plugins::arrowIsVisible(scene_manager_));
  EXPECT_THAT(arrows, SizeIs(1));
  rviz_default_plugins::assertArrowWithTransform(
    scene_manager_, Ogre::Vector3(1, 2, 3), Ogre::Vector3(1, 1, 1), expected_orientation);
}

TEST_F(PoseArrayDisplayFixture, processMessage_sets_axes_correctly) {
  mockValidTransform();
  auto msg = createMessageWithOnePose();

  display_->setShape("Axes");
  display_->processMessage(msg);

  auto frames = rviz_default_plugins::findAllAxes(scene_manager_->getRootSceneNode());

  auto expected_orientation = Ogre::Quaternion(0, 1, 0, 1);
  expected_orientation.normalise();

  expectAxesAreVisible(frames[0]);
  EXPECT_THAT(frames, SizeIs(1));
  EXPECT_VECTOR3_EQ(Ogre::Vector3(1, 2, 3), frames[0]->getPosition());
  EXPECT_QUATERNION_EQ(expected_orientation, frames[0]->getOrientation());
}

TEST_F(PoseArrayDisplayFixture, processMessage_updates_the_display_correctly_after_shape_change) {
  mockValidTransform();
  auto msg = createMessageWithOnePose();
  display_->setShape("Arrow (3D)");
  display_->processMessage(msg);

  auto arrows = rviz_default_plugins::findAllArrows(scene_manager_->getRootSceneNode());
  auto frames = rviz_default_plugins::findAllAxes(scene_manager_->getRootSceneNode());
  EXPECT_THAT(arrows, SizeIs(1));
  EXPECT_EQ(0, manual_object_->getBoundingRadius());
  EXPECT_THAT(frames, SizeIs(0));

  display_->setShape("Axes");
  display_->processMessage(msg);
  auto post_update_arrows = rviz_default_plugins::findAllArrows(scene_manager_->getRootSceneNode());
  auto post_update_frames = rviz_default_plugins::findAllAxes(scene_manager_->getRootSceneNode());
  EXPECT_THAT(post_update_frames, SizeIs(1));
  EXPECT_EQ(0, manual_object_->getBoundingRadius());
  EXPECT_THAT(post_update_arrows, SizeIs(0));
}
