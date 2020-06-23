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

#include <OgreEntity.h>
#include <OgreMesh.h>

#include "visualization_msgs/msg/marker.hpp"
#include "rviz_rendering/objects/arrow.hpp"
#include "rviz_rendering/objects/shape.hpp"

#include "rviz_default_plugins/displays/marker/markers/arrow_marker.hpp"

#include "../../../scene_graph_introspection.hpp"
#include "markers_test_fixture.hpp"
#include "../marker_messages.hpp"

using namespace ::testing;  // NOLINT

// default orientation is set to (0.5, -0.5, -0.5, -0.5) by arrow marker and arrow.
const auto default_arrow_orientation_ = Ogre::Quaternion(0.5f, -0.5f, -0.5f, -0.5f);
const auto default_arrow_position_ = Ogre::Vector3(0, 0, 0);
const auto default_arrow_scale_ = Ogre::Vector3(1, 0.2f, 0.2f);

TEST_F(MarkersTestFixture, setMessage_makes_the_scene_node_invisible_if_invalid_transform) {
  marker_ = makeMarker<rviz_default_plugins::displays::markers::ArrowMarker>();
  EXPECT_CALL(*frame_manager_, transform(_, _, _, _, _)).WillOnce(Return(false));  // NOLINT

  marker_->setMessage(createDefaultMessage(visualization_msgs::msg::Marker::ARROW));

  EXPECT_FALSE(rviz_default_plugins::arrowIsVisible(scene_manager_->getRootSceneNode()));
}

TEST_F(MarkersTestFixture, incomplete_message_sets_scene_node_to_not_visible) {
  marker_ = makeMarker<rviz_default_plugins::displays::markers::ArrowMarker>();

  auto incomplete_message = createMessageWithPoints(visualization_msgs::msg::Marker::ARROW);
  incomplete_message.points.pop_back();

  marker_->setMessage(incomplete_message);

  EXPECT_FALSE(rviz_default_plugins::arrowIsVisible(scene_manager_->getRootSceneNode()));
}

TEST_F(MarkersTestFixture, setMessage_sets_positions_and_orientations_correctly) {
  marker_ = makeMarker<rviz_default_plugins::displays::markers::ArrowMarker>();
  mockValidTransform();

  marker_->setMessage(createDefaultMessage(visualization_msgs::msg::Marker::ARROW));

  EXPECT_THAT(marker_->getPosition(), Vector3Eq(Ogre::Vector3(0, 1, 0)));
  EXPECT_THAT(marker_->getOrientation(), QuaternionEq(Ogre::Quaternion(0, 0, 1, 0)));

  rviz_default_plugins::assertArrowWithTransform(
    scene_manager_, default_arrow_position_, default_arrow_scale_, default_arrow_orientation_);
}

TEST_F(MarkersTestFixture, setMessage_sets_positions_and_orientations_from_points_correctly) {
  marker_ = makeMarker<rviz_default_plugins::displays::markers::ArrowMarker>();
  mockValidTransform();

  auto message = createMessageWithPoints(visualization_msgs::msg::Marker::ARROW);
  marker_->setMessage(message);

  auto first_point = Ogre::Vector3(
    message.points[0].x, message.points[0].y, message.points[0].z);
  auto second_point = Ogre::Vector3(
    message.points[1].x, message.points[1].y, message.points[1].z);
  auto direction = second_point - first_point;
  direction.normalise();
  auto expected_arrow_orientation = Ogre::Vector3::NEGATIVE_UNIT_Z.getRotationTo(direction) *
    Ogre::Quaternion(Ogre::Degree(-90), Ogre::Vector3::UNIT_X);
  Ogre::Vector3 expected_arrow_scale(1, 1, 1);

  EXPECT_THAT(marker_->getPosition(), Vector3Eq(Ogre::Vector3(0, 1, 0)));
  EXPECT_THAT(marker_->getOrientation(), QuaternionEq(Ogre::Quaternion(0, 0, 1, 0)));

  rviz_default_plugins::assertArrowWithTransform(
    scene_manager_, first_point, expected_arrow_scale, expected_arrow_orientation);
}

TEST_F(MarkersTestFixture, setMessage_ignores_points_if_thery_are_more_than_two) {
  marker_ = makeMarker<rviz_default_plugins::displays::markers::ArrowMarker>();
  mockValidTransform();

  geometry_msgs::msg::Point point;
  auto message = createMessageWithPoints(visualization_msgs::msg::Marker::ARROW);
  message.points.push_back(point);

  marker_->setMessage(message);

  rviz_default_plugins::assertArrowWithTransform(
    scene_manager_, default_arrow_position_, default_arrow_scale_, default_arrow_orientation_);
}

TEST_F(MarkersTestFixture, setMessage_ignores_old_message) {
  marker_ = makeMarker<rviz_default_plugins::displays::markers::ArrowMarker>();
  mockValidTransform();

  marker_->setMessage(createMessageWithPoints(visualization_msgs::msg::Marker::ARROW));
  marker_->setMessage(createDefaultMessage(visualization_msgs::msg::Marker::ARROW));

  rviz_default_plugins::assertArrowWithTransform(
    scene_manager_, default_arrow_position_, default_arrow_scale_, default_arrow_orientation_);
}
