/*
 * Copyright (c) 2018, Bosch Software Innovations GmbH.
 * Copyright (c) 2020, Sarthak Mittal.
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
#include <vector>

#include <OgreRoot.h>
#include <OgreEntity.h>
#include <OgreManualObject.h>

#include "rviz_common/properties/float_property.hpp"

#include "rviz_default_plugins/displays/particle_cloud/particle_cloud_display.hpp"

#include "test/rviz_rendering/scene_graph_introspection.hpp"
#include "../display_test_fixture.hpp"
#include "../../scene_graph_introspection_helper.hpp"

using namespace ::testing;  // NOLINT

class ParticleCloudDisplayFixture : public DisplayTestFixture
{
public:
  ParticleCloudDisplayFixture()
  {
    display_ = std::make_unique<rviz_default_plugins::displays::ParticleCloudDisplay>(
      context_.get(),
      scene_manager_->getRootSceneNode()->createChildSceneNode());

    for (int i = 2; i < 6; i++) {
      common_arrow_properties_.push_back(display_->childAt(i));
    }
  }

  std::unique_ptr<rviz_default_plugins::displays::ParticleCloudDisplay> display_;
  std::vector<rviz_common::properties::Property *> common_arrow_properties_;
};

nav2_msgs::msg::ParticleCloud::SharedPtr createMessageWithOnePose()
{
  auto message = std::make_shared<nav2_msgs::msg::ParticleCloud>();
  message->header = std_msgs::msg::Header();
  message->header.frame_id = "particle_cloud_frame";
  message->header.stamp = rclcpp::Clock().now();

  nav2_msgs::msg::Particle particle;
  particle.pose.position.x = 1;
  particle.pose.position.y = 2;
  particle.pose.position.z = 3;
  particle.pose.orientation.x = 1;
  particle.pose.orientation.y = 0;
  particle.pose.orientation.z = 1;
  particle.pose.orientation.w = 0;
  particle.weight = 1;

  message->particles.push_back(particle);

  return message;
}

TEST_F(ParticleCloudDisplayFixture, constructor_set_all_the_properties_in_the_right_order) {
  EXPECT_THAT(display_->childAt(2)->getNameStd(), Eq("Color"));
  EXPECT_THAT(display_->childAt(3)->getNameStd(), Eq("Alpha"));
  EXPECT_THAT(display_->childAt(4)->getNameStd(), Eq("Min Arrow Length"));
  EXPECT_THAT(display_->childAt(5)->getNameStd(), Eq("Max Arrow Length"));
}

TEST_F(ParticleCloudDisplayFixture, setTransform_with_invalid_message_returns_early) {
  mockValidTransform();
  auto msg = createMessageWithOnePose();
  msg->particles[0].pose.position.x = nan("NaN");
  display_->processMessage(msg);

  auto arrows_3d = rviz_rendering::findAllArrows(scene_manager_->getRootSceneNode());
  auto axes = rviz_rendering::findAllAxes(scene_manager_->getRootSceneNode());
  auto manual_object =
    rviz_rendering::findOneManualObject(scene_manager_->getRootSceneNode());

  // the default position and orientation of the scene node are (0, 0, 0) and (1, 0, 0, 0)
  EXPECT_THAT(display_->getSceneNode()->getPosition(), Vector3Eq(Ogre::Vector3(0, 0, 0)));
  EXPECT_THAT(
    display_->getSceneNode()->getOrientation(), QuaternionEq(Ogre::Quaternion(1, 0, 0, 0)));
  EXPECT_THAT(manual_object->getBoundingRadius(), FloatEq(0));
  EXPECT_THAT(arrows_3d, SizeIs(0));
  EXPECT_THAT(axes, SizeIs(0));
}

TEST_F(ParticleCloudDisplayFixture, setTransform_with_invalid_transform_returns_early) {
  EXPECT_CALL(*frame_manager_, getTransform(_, _, _, _)).WillOnce(Return(false));

  auto msg = createMessageWithOnePose();
  display_->processMessage(msg);

  auto arrows_3d = rviz_rendering::findAllArrows(scene_manager_->getRootSceneNode());
  auto axes = rviz_rendering::findAllAxes(scene_manager_->getRootSceneNode());
  auto manual_object = rviz_rendering::findOneManualObject(scene_manager_->getRootSceneNode());

  // the default position and orientation of the scene node are (0, 0, 0) and (1, 0, 0, 0)
  EXPECT_THAT(display_->getSceneNode()->getPosition(), Vector3Eq(Ogre::Vector3(0, 0, 0)));
  EXPECT_THAT(
    display_->getSceneNode()->getOrientation(), QuaternionEq(Ogre::Quaternion(1, 0, 0, 0)));
  EXPECT_THAT(manual_object->getBoundingRadius(), FloatEq(0));
  EXPECT_THAT(arrows_3d, SizeIs(0));
  EXPECT_THAT(axes, SizeIs(0));
}

TEST_F(ParticleCloudDisplayFixture, setTransform_sets_node_position_and_orientation_correctly) {
  mockValidTransform();
  auto msg = createMessageWithOnePose();
  display_->processMessage(msg);

  EXPECT_THAT(display_->getSceneNode()->getPosition(), Vector3Eq(Ogre::Vector3(0, 1, 0)));
  EXPECT_THAT(
    display_->getSceneNode()->getOrientation(), QuaternionEq(Ogre::Quaternion(0, 0, 1, 0)));
}

TEST_F(ParticleCloudDisplayFixture, processMessage_sets_manualObject_correctly) {
  mockValidTransform();
  auto msg = createMessageWithOnePose();
  display_->processMessage(msg);

  auto manual_object = rviz_rendering::findOneManualObject(scene_manager_->getRootSceneNode());
  auto manual_objectbounding_radius = 4.17732;
  EXPECT_THAT(manual_object->getBoundingRadius(), FloatEq(manual_objectbounding_radius));
  EXPECT_THAT(
    manual_object->getBoundingBox().getCenter(), Vector3Eq(Ogre::Vector3(0.85f, 2, 3.3f)));
}

TEST_F(ParticleCloudDisplayFixture, processMessage_sets_arrows3d_correctly) {
  mockValidTransform();
  auto msg = createMessageWithOnePose();

  display_->setShape("Arrow (3D)");
  display_->processMessage(msg);

  auto arrows = rviz_rendering::findAllArrows(scene_manager_->getRootSceneNode());

  // The orientation is first manipulated by the displays and then in setOrientation() in arrow.cpp
  auto expected_orientation =
    Ogre::Quaternion(0, 1, 0, 1) *
    Ogre::Quaternion(Ogre::Degree(-90), Ogre::Vector3::UNIT_Y) *
    Ogre::Quaternion(Ogre::Degree(-90), Ogre::Vector3::UNIT_X);
  expected_orientation.normalise();

  EXPECT_TRUE(rviz_default_plugins::arrowIsVisible(scene_manager_->getRootSceneNode()));
  EXPECT_THAT(arrows, SizeIs(1));
  rviz_default_plugins::assertArrowWithTransform(
    scene_manager_, Ogre::Vector3(1, 2, 3), Ogre::Vector3(1, 1, 1), expected_orientation);
}

TEST_F(ParticleCloudDisplayFixture, processMessage_sets_axes_correctly) {
  mockValidTransform();
  auto msg = createMessageWithOnePose();

  display_->setShape("Axes");
  display_->processMessage(msg);

  auto frames = rviz_rendering::findAllAxes(scene_manager_->getRootSceneNode());

  auto expected_orientation = Ogre::Quaternion(0, 1, 0, 1);
  expected_orientation.normalise();

  EXPECT_TRUE(rviz_default_plugins::axesAreVisible(frames[0]));
  EXPECT_THAT(frames, SizeIs(1));
  EXPECT_THAT(frames[0]->getPosition(), Vector3Eq(Ogre::Vector3(1, 2, 3)));
  EXPECT_THAT(frames[0]->getOrientation(), QuaternionEq(expected_orientation));
}

TEST_F(
  ParticleCloudDisplayFixture,
  processMessage_updates_the_display_correctly_after_shape_change) {
  mockValidTransform();
  auto msg = createMessageWithOnePose();
  display_->setShape("Arrow (3D)");
  display_->processMessage(msg);

  auto arrows = rviz_rendering::findAllArrows(scene_manager_->getRootSceneNode());
  auto frames = rviz_rendering::findAllAxes(scene_manager_->getRootSceneNode());
  auto manual_object = rviz_rendering::findOneManualObject(scene_manager_->getRootSceneNode());
  EXPECT_THAT(arrows, SizeIs(1));
  EXPECT_THAT(manual_object->getBoundingRadius(), FloatEq(0));
  EXPECT_THAT(frames, SizeIs(0));

  display_->setShape("Axes");
  display_->processMessage(msg);
  auto post_update_arrows = rviz_rendering::findAllArrows(scene_manager_->getRootSceneNode());
  auto post_update_frames = rviz_rendering::findAllAxes(scene_manager_->getRootSceneNode());
  EXPECT_THAT(post_update_frames, SizeIs(1));
  EXPECT_THAT(manual_object->getBoundingRadius(), FloatEq(0));
  EXPECT_THAT(post_update_arrows, SizeIs(0));
}
