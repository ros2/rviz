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
 *     * Neither the name of the copyright holders nor the names of its
 *       contributors may be used to endorse or promote products derived from
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

#include <OgreEntity.h>
#include <OgreSceneNode.h>

#include "rviz_default_plugins/displays/range/range_display.hpp"
#include "../display_test_fixture.hpp"
#include "../../scene_graph_introspection.hpp"

using namespace ::testing;  // NOLINT

class RangeDisplayFixture : public DisplayTestFixture
{
public:
  RangeDisplayFixture()
  {
    display_ = std::make_unique<rviz_default_plugins::displays::RangeDisplay>(context_.get());
  }

  std::unique_ptr<rviz_default_plugins::displays::RangeDisplay> display_;
};

sensor_msgs::msg::Range::ConstSharedPtr createRangeMessage(float range = 5)
{
  auto message = std::make_shared<sensor_msgs::msg::Range>();
  message->header = std_msgs::msg::Header();
  message->header.frame_id = "range_frame";
  message->header.stamp = rclcpp::Clock().now();

  message->radiation_type = sensor_msgs::msg::Range::INFRARED;
  message->range = range;
  message->min_range = 0;
  message->max_range = 6;
  message->field_of_view = 1.0471f;

  return message;
}

TEST_F(RangeDisplayFixture, updateBufferLength_creates_right_number_of_cones) {
  auto cones = rviz_default_plugins::findAllCones(scene_manager_->getRootSceneNode());
  EXPECT_THAT(cones, SizeIs(1));

  display_->findProperty("Buffer Length")->setValue(3);
  cones = rviz_default_plugins::findAllCones(scene_manager_->getRootSceneNode());
  EXPECT_THAT(cones, SizeIs(3));
}

TEST_F(RangeDisplayFixture, processMessage_returns_early_if_transform_is_missing) {
  auto message = createRangeMessage();
  display_->processMessage(message);

  auto cones = rviz_default_plugins::findAllCones(scene_manager_->getRootSceneNode());
  ASSERT_THAT(cones, SizeIs(1));

  auto cone_position = cones[0]->getParentSceneNode()->getParentSceneNode()->getPosition();
  auto cone_orientation = cones[0]->getParentSceneNode()->getParentSceneNode()->getOrientation();
  auto cone_scale = cones[0]->getParentSceneNode()->getParentSceneNode()->getScale();
  EXPECT_THAT(cone_position, Vector3Eq(Ogre::Vector3(0, 0, 0)));
  EXPECT_THAT(cone_orientation, QuaternionEq(Ogre::Quaternion(1, 0, 0, 0)));
  EXPECT_THAT(cone_scale, Vector3Eq(Ogre::Vector3(0, 0, 0)));
}

TEST_F(RangeDisplayFixture, processMessage_sets_cones_correctly) {
  mockValidTransform();
  auto message = createRangeMessage();
  display_->processMessage(message);

  auto cones = rviz_default_plugins::findAllCones(scene_manager_->getRootSceneNode());
  ASSERT_THAT(cones, SizeIs(1));

  auto cone_position = cones[0]->getParentSceneNode()->getParentSceneNode()->getPosition();
  auto cone_orientation = cones[0]->getParentSceneNode()->getParentSceneNode()->getOrientation();
  auto cone_scale = cones[0]->getParentSceneNode()->getParentSceneNode()->getScale();
  EXPECT_THAT(cone_position, Vector3Eq(Ogre::Vector3(0, 1, 0)));
  EXPECT_THAT(cone_orientation, QuaternionEq(Ogre::Quaternion(0, 0, 1, 0)));
  EXPECT_THAT(cone_scale, Vector3Eq(Ogre::Vector3(5.77285f, 5, 5.77285f)));
}
