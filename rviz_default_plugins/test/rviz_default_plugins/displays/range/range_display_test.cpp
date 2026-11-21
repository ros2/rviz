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

#include <algorithm>
#include <memory>
#include <vector>

#include <OgreEntity.h>
#include <OgreSceneNode.h>

#include "rclcpp/clock.hpp"

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

sensor_msgs::msg::Range::ConstSharedPtr createRangeMessage(float range = 5, float variance = 0)
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
  message->variance = variance;

  return message;
}

std::vector<Ogre::Vector3> coneScales(Ogre::SceneNode * root)
{
  std::vector<Ogre::Vector3> scales;
  for (const auto & cone : rviz_default_plugins::findAllCones(root)) {
    scales.push_back(cone->getParentSceneNode()->getParentSceneNode()->getScale());
  }
  return scales;
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

TEST_F(RangeDisplayFixture, enabling_variance_adds_a_near_and_far_cone_per_buffer_slot) {
  // updateBufferLength_creates_right_number_of_cones already covers the default,
  // switched-off case: the band must add nothing until it is enabled.
  display_->findProperty("Show Variance")->setValue(true);
  EXPECT_THAT(rviz_default_plugins::findAllCones(scene_manager_->getRootSceneNode()), SizeIs(3));

  display_->findProperty("Buffer Length")->setValue(2);
  EXPECT_THAT(rviz_default_plugins::findAllCones(scene_manager_->getRootSceneNode()), SizeIs(6));

  display_->findProperty("Show Variance")->setValue(false);
  EXPECT_THAT(rviz_default_plugins::findAllCones(scene_manager_->getRootSceneNode()), SizeIs(2));
}

TEST_F(RangeDisplayFixture, processMessage_scales_the_variance_band_to_one_sigma) {
  mockValidTransform();
  display_->findProperty("Show Variance")->setValue(true);

  // sigma = sqrt(0.25) = 0.5, so the band spans 4.5m to 5.5m.
  display_->processMessage(createRangeMessage(5, 0.25f));

  auto scales = coneScales(scene_manager_->getRootSceneNode());
  ASSERT_THAT(scales, SizeIs(3));
  EXPECT_THAT(scales, Contains(Vector3Eq(Ogre::Vector3(5.77285f, 5, 5.77285f))));
  EXPECT_THAT(scales, Contains(Vector3Eq(Ogre::Vector3(5.195565f, 4.5f, 5.195565f))));
  EXPECT_THAT(scales, Contains(Vector3Eq(Ogre::Vector3(6.350135f, 5.5f, 6.350135f))));
}

TEST_F(RangeDisplayFixture, processMessage_hides_the_variance_band_when_variance_is_unknown) {
  mockValidTransform();
  display_->findProperty("Show Variance")->setValue(true);

  // sensor_msgs/Range documents 0 as "variance unknown".
  display_->processMessage(createRangeMessage(5, 0));

  auto scales = coneScales(scene_manager_->getRootSceneNode());
  ASSERT_THAT(scales, SizeIs(3));
  EXPECT_THAT(scales, Contains(Vector3Eq(Ogre::Vector3(5.77285f, 5, 5.77285f))));
  EXPECT_THAT(
    std::count(scales.begin(), scales.end(), Ogre::Vector3(0, 0, 0)), Eq(2));
}

TEST_F(RangeDisplayFixture, processMessage_hides_the_near_edge_when_sigma_exceeds_the_range) {
  mockValidTransform();
  display_->findProperty("Show Variance")->setValue(true);

  // sigma = 3 against a 2m range puts the near edge behind the sensor, so only the
  // far edge at 5m is drawable.
  display_->processMessage(createRangeMessage(2, 9.0f));

  auto scales = coneScales(scene_manager_->getRootSceneNode());
  ASSERT_THAT(scales, SizeIs(3));
  EXPECT_THAT(scales, Contains(Vector3Eq(Ogre::Vector3(2.30914f, 2, 2.30914f))));
  EXPECT_THAT(scales, Contains(Vector3Eq(Ogre::Vector3(5.77285f, 5, 5.77285f))));
  EXPECT_THAT(
    std::count(scales.begin(), scales.end(), Ogre::Vector3(0, 0, 0)), Eq(1));
}
