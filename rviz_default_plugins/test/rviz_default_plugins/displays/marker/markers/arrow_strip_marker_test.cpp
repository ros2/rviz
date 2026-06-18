// Copyright (c) 2018, Bosch Software Innovations GmbH.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#include <gmock/gmock.h>

#include <memory>
#include <string>

#include <OgreEntity.h>
#include <OgreMesh.h>

#include "visualization_msgs/msg/marker.hpp"
#include "rviz_rendering/objects/arrow.hpp"
#include "rviz_rendering/objects/shape.hpp"

#include "rviz_default_plugins/displays/marker/markers/arrow_strip_marker.hpp"

#include "../../../scene_graph_introspection.hpp"
#include "markers_test_fixture.hpp"
#include "../marker_messages.hpp"

using namespace ::testing;  // NOLINT

// default orientation is set to (0.5, -0.5, -0.5, -0.5) by arrow marker and arrow.
const auto default_arrow_orientation_ = Ogre::Quaternion(0.5f, -0.5f, -0.5f, -0.5f);
const auto default_arrow_position_ = Ogre::Vector3(0, 0, 0);
const auto default_arrow_scale_ = Ogre::Vector3(1, 0.2f, 0.2f);

TEST_F(MarkersTestFixture, setMessage_makes_arrow_strip_invisible_if_invalid_transform) {
  marker_ = makeMarker<rviz_default_plugins::displays::markers::ArrowStripMarker>();
  EXPECT_CALL(*frame_manager_, transform(_, _, _, _, _)).WillOnce(Return(false));  // NOLINT

  marker_->setMessage(createDefaultMessage(visualization_msgs::msg::Marker::ARROW_STRIP));

  EXPECT_TRUE(rviz_default_plugins::noArrowsAreVisible(scene_manager_->getRootSceneNode()));
}

TEST_F(MarkersTestFixture, incomplete_message_sets_arrow_strip_to_not_visible) {
  marker_ = makeMarker<rviz_default_plugins::displays::markers::ArrowStripMarker>();

  auto incomplete_message = createMessageWithTwoPoints(visualization_msgs::msg::Marker::ARROW_STRIP);
  incomplete_message.points.pop_back();

  marker_->setMessage(incomplete_message);

  EXPECT_TRUE(rviz_default_plugins::noArrowsAreVisible(scene_manager_->getRootSceneNode()));
}

TEST_F(MarkersTestFixture, setMessage_sets_arrow_strip_positions_and_orientations_correctly) {
  marker_ = makeMarker<rviz_default_plugins::displays::markers::ArrowStripMarker>();
  mockValidTransform();

  marker_->setMessage(createDefaultMessage(visualization_msgs::msg::Marker::ARROW_STRIP));

  EXPECT_THAT(marker_->getPosition(), Vector3Eq(Ogre::Vector3(0, 1, 0)));
  EXPECT_THAT(marker_->getOrientation(), QuaternionEq(Ogre::Quaternion(0, 0, 1, 0)));

  rviz_default_plugins::assertArrowWithTransform(
    scene_manager_, default_arrow_position_, default_arrow_scale_, default_arrow_orientation_);
}

TEST_F(MarkersTestFixture, setMessage_sets_arrow_strip_positions_and_orientations_from_two_points_correctly) {
  marker_ = makeMarker<rviz_default_plugins::displays::markers::ArrowStripMarker>();
  mockValidTransform();

  auto message = createMessageWithTwoPoints(visualization_msgs::msg::Marker::ARROW_STRIP);
  marker_->setMessage(message);

  auto p1 = Ogre::Vector3(
    message.points[0].x, message.points[0].y, message.points[0].z);
  auto p2 = Ogre::Vector3(
    message.points[1].x, message.points[1].y, message.points[1].z);

  auto direction = p2 - p1;
  direction.normalise();

  auto expected_arrow_orientation = Ogre::Vector3::NEGATIVE_UNIT_Z.getRotationTo(direction) *
    Ogre::Quaternion(Ogre::Degree(-90), Ogre::Vector3::UNIT_X);

  Ogre::Vector3 expected_arrow_scale(1, 1, 1);

  EXPECT_THAT(marker_->getPosition(), Vector3Eq(Ogre::Vector3(0, 1, 0)));
  EXPECT_THAT(marker_->getOrientation(), QuaternionEq(Ogre::Quaternion(0, 0, 1, 0)));

  rviz_default_plugins::assertArrowWithTransform(
    scene_manager_, p1, expected_arrow_scale, expected_arrow_orientation);
}

TEST_F(MarkersTestFixture, setMessage_sets_arrow_strip_positions_and_orientations_from_three_points_correctly) {
  marker_ = makeMarker<rviz_default_plugins::displays::markers::ArrowStripMarker>();
  mockValidTransform();

  auto message = createMessageWithThreePoints(visualization_msgs::msg::Marker::ARROW_STRIP);
  marker_->setMessage(message);

  auto p1 = Ogre::Vector3(
    message.points[0].x, message.points[0].y, message.points[0].z);
  auto p2 = Ogre::Vector3(
    message.points[1].x, message.points[1].y, message.points[1].z);
  auto p3 = Ogre::Vector3(
    message.points[2].x, message.points[2].y, message.points[2].z);

  auto d1 = p2 - p1;
  auto d2 = p3 - p2;

  d1.normalise();
  d2.normalise();

  auto q1 = Ogre::Vector3::NEGATIVE_UNIT_Z.getRotationTo(d1) *
    Ogre::Quaternion(Ogre::Degree(-90), Ogre::Vector3::UNIT_X);
  auto q2 = Ogre::Vector3::NEGATIVE_UNIT_Z.getRotationTo(d2) *
    Ogre::Quaternion(Ogre::Degree(-90), Ogre::Vector3::UNIT_X);
  
  Ogre::Vector3 expected_arrow_scale(1, 1, 1);

  EXPECT_THAT(marker_->getPosition(), Vector3Eq(Ogre::Vector3(0, 1, 0)));
  EXPECT_THAT(marker_->getOrientation(), QuaternionEq(Ogre::Quaternion(0, 0, 1, 0)));
  
  std::array<Ogre::Vector3, 2> expected_arrow_points = {p1, p2};
  std::array<Ogre::Quaternion, 2> expected_arrow_orientations = {q1, q2};
  std::array<Ogre::Vector3, 2> expected_arrow_scales = {expected_arrow_scale, expected_arrow_scale};

  rviz_default_plugins::assertArrowsWithTransforms<2>(
    scene_manager_, expected_arrow_points, expected_arrow_scales, expected_arrow_orientations);
}

TEST_F(MarkersTestFixture, setMessage_arrow_strip_ignores_old_message) {
  marker_ = makeMarker<rviz_default_plugins::displays::markers::ArrowStripMarker>();
  mockValidTransform();

  marker_->setMessage(createMessageWithTwoPoints(visualization_msgs::msg::Marker::ARROW_STRIP));
  marker_->setMessage(createDefaultMessage(visualization_msgs::msg::Marker::ARROW_STRIP));
  
  rviz_default_plugins::assertArrowWithTransform(
    scene_manager_, default_arrow_position_, default_arrow_scale_, default_arrow_orientation_);
}
