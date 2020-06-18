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

#include <OgreBillboardChain.h>

#include "visualization_msgs/msg/marker.hpp"
#include "rviz_rendering/objects/arrow.hpp"

#include "rviz_default_plugins/displays/marker/markers/line_list_marker.hpp"
#include "rviz_default_plugins/displays/marker/markers/line_strip_marker.hpp"

#include "../../../scene_graph_introspection.hpp"
#include "markers_test_fixture.hpp"
#include "../marker_messages.hpp"

using namespace ::testing;  // NOLINT

void assertColorEquals(
  std_msgs::msg::ColorRGBA color, Ogre::BillboardChain * billboard_chain, int32_t element)
{
  EXPECT_THAT(billboard_chain->getChainElement(0, element).colour.r, Eq(color.r));
  EXPECT_THAT(billboard_chain->getChainElement(0, element).colour.g, Eq(color.g));
  EXPECT_THAT(billboard_chain->getChainElement(0, element).colour.b, Eq(color.b));
  EXPECT_THAT(billboard_chain->getChainElement(0, element).colour.a, Eq(color.a));
}

TEST_F(MarkersTestFixture, setMessage_does_not_add_anything_when_no_points_are_provided) {
  marker_ = makeMarker<rviz_default_plugins::displays::markers::LineListMarker>();
  mockValidTransform();

  marker_->setMessage(createDefaultMessage(visualization_msgs::msg::Marker::LINE_LIST));

  auto billboard_chain = rviz_default_plugins::findOneBillboardChain(
    scene_manager_->getRootSceneNode());
  ASSERT_TRUE(billboard_chain);
  ASSERT_TRUE(billboard_chain->isVisible());
  EXPECT_THAT(billboard_chain->getNumberOfChains(), Eq(1u));
  EXPECT_THAT(billboard_chain->getNumChainElements(0), Eq(0u));
}

TEST_F(MarkersTestFixture, setMessage_sets_billboard_line_invisible_when_transform_fails) {
  marker_ = makeMarker<rviz_default_plugins::displays::markers::LineListMarker>();

  Ogre::Vector3 position(0, 1, 0);
  Ogre::Quaternion orientation(0, 0, 1, 0);
  EXPECT_CALL(*frame_manager_, transform(_, _, _, _, _)).WillOnce(Return(false));  // NOLINT

  marker_->setMessage(createDefaultMessage(visualization_msgs::msg::Marker::LINE_LIST));

  auto billboard_chain = rviz_default_plugins::findOneBillboardChain(
    scene_manager_->getRootSceneNode());
  ASSERT_TRUE(billboard_chain);
  EXPECT_FALSE(billboard_chain->isVisible());
}

TEST_F(MarkersTestFixture, setMessage_sets_correct_position_and_orientation) {
  marker_ = makeMarker<rviz_default_plugins::displays::markers::LineListMarker>();
  mockValidTransform();

  marker_->setMessage(createDefaultMessage(visualization_msgs::msg::Marker::LINE_LIST));

  EXPECT_THAT(marker_->getPosition(), Vector3Eq(Ogre::Vector3(0, 1, 0)));
  EXPECT_THAT(marker_->getOrientation(), QuaternionEq(Ogre::Quaternion(0, 0, 1, 0)));
}

TEST_F(MarkersTestFixture, setMessage_does_not_show_billboard_line_if_uneven_number_of_points) {
  marker_ = makeMarker<rviz_default_plugins::displays::markers::LineListMarker>();
  mockValidTransform();

  auto message = createDefaultMessage(visualization_msgs::msg::Marker::LINE_LIST);
  message.points.push_back(create_point(0, 0, 0));

  marker_->setMessage(message);

  auto billboard_chain = rviz_default_plugins::findOneBillboardChain(
    scene_manager_->getRootSceneNode());
  ASSERT_TRUE(billboard_chain);
  EXPECT_TRUE(billboard_chain->isVisible());
  EXPECT_THAT(billboard_chain->getNumberOfChains(), Eq(1u));
  EXPECT_THAT(billboard_chain->getNumChainElements(0), Eq(0u));
}

TEST_F(MarkersTestFixture, setMessage_clears_marker_upon_new_message) {
  marker_ = makeMarker<rviz_default_plugins::displays::markers::LineListMarker>();
  mockValidTransform();

  auto message = createDefaultMessage(visualization_msgs::msg::Marker::LINE_LIST);
  message.points.push_back(create_point(0, 0, 0));
  message.points.push_back(create_point(2, 1, 1));
  marker_->setMessage(message);

  marker_->setMessage(createDefaultMessage(visualization_msgs::msg::Marker::LINE_LIST));

  auto billboard_chain = rviz_default_plugins::findOneBillboardChain(
    scene_manager_->getRootSceneNode());
  ASSERT_TRUE(billboard_chain);
  EXPECT_TRUE(billboard_chain->isVisible());
  EXPECT_THAT(billboard_chain->getNumberOfChains(), Eq(1u));
  EXPECT_THAT(billboard_chain->getNumChainElements(0), Eq(0u));
}

TEST_F(MarkersTestFixture, setMessage_adds_billboard_line_with_one_color) {
  marker_ = makeMarker<rviz_default_plugins::displays::markers::LineListMarker>();
  mockValidTransform();

  auto message = createDefaultMessage(visualization_msgs::msg::Marker::LINE_LIST);
  geometry_msgs::msg::Point first_point = create_point(0, 0, 0);
  geometry_msgs::msg::Point second_point = create_point(1, 1, 0);

  message.points.push_back(first_point);
  message.points.push_back(second_point);

  marker_->setMessage(message);

  auto billboard_chain = rviz_default_plugins::findOneBillboardChain(
    scene_manager_->getRootSceneNode());
  ASSERT_TRUE(billboard_chain);
  EXPECT_TRUE(billboard_chain->isVisible());
  EXPECT_THAT(billboard_chain->getNumberOfChains(), Eq(1u));
  EXPECT_THAT(billboard_chain->getNumChainElements(0), Eq(2u));
  EXPECT_THAT(
    billboard_chain->getChainElement(0, 1).position,
    Vector3Eq(Ogre::Vector3(first_point.x, first_point.y, first_point.z)));
  EXPECT_THAT(
    billboard_chain->getChainElement(0, 0).position,
    Vector3Eq(Ogre::Vector3(second_point.x, second_point.y, second_point.z)));
  assertColorEquals(message.color, billboard_chain, 0);
  assertColorEquals(message.color, billboard_chain, 1);
}

TEST_F(
  MarkersTestFixture,
  setMessage_adds_billboard_line_with_many_colors_if_all_points_have_color_information) {
  marker_ = makeMarker<rviz_default_plugins::displays::markers::LineListMarker>();
  mockValidTransform();

  auto message = createDefaultMessage(visualization_msgs::msg::Marker::LINE_LIST);
  geometry_msgs::msg::Point first_point = create_point(0, 0, 0);
  std_msgs::msg::ColorRGBA first_point_color = color(0.5f, 0.6f, 0.7f, 0.5f);
  message.points.push_back(first_point);
  message.colors.push_back(first_point_color);

  geometry_msgs::msg::Point second_point = create_point(1, 1, 0);
  std_msgs::msg::ColorRGBA second_point_color = color(0.3f, 0.4f, 0.5f, 0.6f);
  message.points.push_back(second_point);
  message.colors.push_back(second_point_color);

  marker_->setMessage(message);

  auto billboard_chain = rviz_default_plugins::findOneBillboardChain(
    scene_manager_->getRootSceneNode());
  ASSERT_TRUE(billboard_chain);
  EXPECT_TRUE(billboard_chain->isVisible());
  EXPECT_THAT(billboard_chain->getNumberOfChains(), Eq(1u));
  EXPECT_THAT(billboard_chain->getNumChainElements(0), Eq(2u));
  EXPECT_THAT(
    billboard_chain->getChainElement(0, 1).position,
    Vector3Eq(Ogre::Vector3(first_point.x, first_point.y, first_point.z)));
  EXPECT_THAT(
    billboard_chain->getChainElement(0, 0).position,
    Vector3Eq(Ogre::Vector3(second_point.x, second_point.y, second_point.z)));
  assertColorEquals(second_point_color, billboard_chain, 0);
  assertColorEquals(first_point_color, billboard_chain, 1);
}

TEST_F(MarkersTestFixture, setMessage_shows_billboard_strip_for_uneven_number_of_points) {
  marker_ = makeMarker<rviz_default_plugins::displays::markers::LineStripMarker>();
  mockValidTransform();

  auto message = createDefaultMessage(visualization_msgs::msg::Marker::LINE_STRIP);

  message.points.push_back(create_point(0, 0, 0));

  marker_->setMessage(message);

  auto billboard_chain = rviz_default_plugins::findOneBillboardChain(
    scene_manager_->getRootSceneNode());
  ASSERT_TRUE(billboard_chain);
  EXPECT_TRUE(billboard_chain->isVisible());
  EXPECT_THAT(billboard_chain->getNumberOfChains(), Eq(1u));
  EXPECT_THAT(billboard_chain->getNumChainElements(0), Eq(1u));
}

TEST_F(MarkersTestFixture, setMessage_adds_many_points_into_same_chain) {
  marker_ = makeMarker<rviz_default_plugins::displays::markers::LineStripMarker>();
  mockValidTransform();
  auto message = createDefaultMessage(visualization_msgs::msg::Marker::LINE_STRIP);
  geometry_msgs::msg::Point first_point = create_point(0, 0, 0);
  geometry_msgs::msg::Point second_point = create_point(1, 1, 0);
  geometry_msgs::msg::Point third_point = create_point(0, 1, 1);
  message.points.push_back(first_point);
  message.points.push_back(second_point);
  message.points.push_back(third_point);

  marker_->setMessage(message);

  auto billboard_chain = rviz_default_plugins::findOneBillboardChain(
    scene_manager_->getRootSceneNode());
  ASSERT_TRUE(billboard_chain);
  EXPECT_TRUE(billboard_chain->isVisible());
  EXPECT_THAT(billboard_chain->getNumberOfChains(), Eq(1u));
  EXPECT_THAT(billboard_chain->getNumChainElements(0), Eq(3u));
  EXPECT_THAT(
    billboard_chain->getChainElement(0, 2).position,
    Vector3Eq(Ogre::Vector3(first_point.x, first_point.y, first_point.z)));
  EXPECT_THAT(
    billboard_chain->getChainElement(0, 1).position,
    Vector3Eq(Ogre::Vector3(second_point.x, second_point.y, second_point.z)));
  EXPECT_THAT(
    billboard_chain->getChainElement(0, 0).position,
    Vector3Eq(Ogre::Vector3(third_point.x, third_point.y, third_point.z)));
}
