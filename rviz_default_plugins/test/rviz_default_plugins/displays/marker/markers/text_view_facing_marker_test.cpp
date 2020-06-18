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
#include <OgreSimpleRenderable.h>

#include "visualization_msgs/msg/marker.hpp"
#include "rviz_rendering/objects/movable_text.hpp"

#include "rviz_default_plugins/displays/marker/markers/text_view_facing_marker.hpp"

#include "../../../scene_graph_introspection.hpp"
#include "markers_test_fixture.hpp"
#include "../marker_messages.hpp"

using namespace ::testing;  // NOLINT

TEST_F(MarkersTestFixture, setMessage_sets_object_invisible_when_transform_fails) {
  marker_ = makeMarker<rviz_default_plugins::displays::markers::TextViewFacingMarker>();
  EXPECT_CALL(*frame_manager_, transform(_, _, _, _, _)).WillOnce(Return(false));  // NOLINT

  marker_->setMessage(createDefaultMessage(visualization_msgs::msg::Marker::TEXT_VIEW_FACING));

  auto movable_text = rviz_default_plugins::findOneMovableText(scene_manager_->getRootSceneNode());
  ASSERT_TRUE(movable_text);

  EXPECT_FALSE(movable_text->isVisible());
}

TEST_F(MarkersTestFixture, setMessage_sets_correct_caption_for_valid_message) {
  marker_ = makeMarker<rviz_default_plugins::displays::markers::TextViewFacingMarker>();
  mockValidTransform();

  auto message = createDefaultMessage(visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
  message.text = "Message to display";
  marker_->setMessage(message);

  auto movable_text = rviz_default_plugins::findOneMovableText(scene_manager_->getRootSceneNode());
  ASSERT_TRUE(movable_text);
  EXPECT_TRUE(movable_text->isVisible());
  EXPECT_THAT(movable_text->getCaption(), StrEq(message.text));
}

TEST_F(MarkersTestFixture, setMessage_sets_correct_transformation_for_valid_message) {
  marker_ = makeMarker<rviz_default_plugins::displays::markers::TextViewFacingMarker>();
  mockValidTransform();

  auto message = createDefaultMessage(visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
  message.text = "Message to display";
  marker_->setMessage(message);

  auto movable_text = rviz_default_plugins::findOneMovableText(scene_manager_->getRootSceneNode());
  ASSERT_TRUE(movable_text);
  EXPECT_THAT(movable_text->getParentSceneNode()->getPosition(), Vector3Eq(Ogre::Vector3(0, 1, 0)));
  EXPECT_THAT(movable_text->getCharacterHeight(), Eq(message.scale.z));
}
