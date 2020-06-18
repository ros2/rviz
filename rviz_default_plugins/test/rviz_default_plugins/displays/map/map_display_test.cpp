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

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <OgreManualObject.h>

#include "rviz_default_plugins/displays/map/map_display.hpp"
#include "../../scene_graph_introspection.hpp"
#include "../display_test_fixture.hpp"

using namespace ::testing;  // NOLINT

class MapTestFixture : public DisplayTestFixture
{
public:
  void SetUp() override
  {
    DisplayTestFixture::SetUp();
    map_display_ = std::make_shared<rviz_default_plugins::displays::MapDisplay>(context_.get());
  }

  void TearDown() override
  {
    map_display_.reset();
    DisplayTestFixture::TearDown();
  }

  nav_msgs::msg::OccupancyGrid::SharedPtr createMapMessage(
    uint32_t width = 50, uint32_t height = 50)
  {
    auto header = std_msgs::msg::Header();
    header.frame_id = "map_frame";
    header.stamp = rclcpp::Clock().now();

    auto meta_data = nav_msgs::msg::MapMetaData();
    meta_data.height = height;
    meta_data.width = width;
    meta_data.map_load_time = rclcpp::Clock().now();
    meta_data.resolution = 1;

    meta_data.origin.position.x = -25;
    meta_data.origin.position.y = -25;
    meta_data.origin.position.z = 0;

    meta_data.origin.orientation.x = 0;
    meta_data.origin.orientation.y = 0;
    meta_data.origin.orientation.z = 0;
    meta_data.origin.orientation.w = 1;

    auto new_data = std::vector<int8_t>();
    for (uint32_t i = 0; i < width; i++) {
      for (uint32_t j = 0; j < height; j++) {
        new_data.emplace_back((i + j) % 101);
      }
    }

    auto occupancy_grid = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    occupancy_grid->header = header;
    occupancy_grid->info = meta_data;
    occupancy_grid->data = new_data;

    return occupancy_grid;
  }

  void expectNoManualObjects()
  {
    map_display_->showMap();
    auto manual_objects = rviz_default_plugins::findAllOgreObjectByType<Ogre::ManualObject>(
      scene_manager_->getRootSceneNode(), "ManualObject");

    EXPECT_THAT(manual_objects, IsEmpty());
  }

  std::shared_ptr<rviz_default_plugins::displays::MapDisplay> map_display_;
};

TEST_F(MapTestFixture, showMap_does_not_print_map_if_map_is_empty) {
  auto invalid_message = createMapMessage();
  invalid_message->info.width = 0;
  map_display_->processMessage(invalid_message);

  expectNoManualObjects();
}

TEST_F(MapTestFixture, showMap_does_not_print_map_if_map_contain_invalid_resolution) {
  auto invalid_message = createMapMessage();
  invalid_message->info.resolution = nan("NaN");
  map_display_->processMessage(invalid_message);

  expectNoManualObjects();
}

TEST_F(MapTestFixture, showMap_does_not_print_map_if_map_contains_invalid_position) {
  auto invalid_message = createMapMessage();
  invalid_message->info.origin.position.x = nan("NaN");
  map_display_->processMessage(invalid_message);

  expectNoManualObjects();
}

TEST_F(MapTestFixture, showMap_does_not_print_map_if_map_sizes_do_not_match_data) {
  auto invalid_message = createMapMessage();
  invalid_message->info.height = 1000;
  invalid_message->info.width = 2000;
  map_display_->processMessage(invalid_message);

  expectNoManualObjects();
}

TEST_F(MapTestFixture, showMap_shows_manual_object_if_current_map_is_valid) {
  mockValidTransform();

  map_display_->processMessage(createMapMessage());

  auto manual_objects = rviz_default_plugins::findAllOgreObjectByType<Ogre::ManualObject>(
    scene_manager_->getRootSceneNode(), "ManualObject");

  ASSERT_THAT(manual_objects, SizeIs(1));
  EXPECT_TRUE(manual_objects[0]->isVisible());
}

TEST_F(MapTestFixture, showMap_defaults_empty_frame_id_to_map) {
  Ogre::Vector3 position(1, 0, 0);
  Ogre::Quaternion orientation(0.707107f, 0.707107f, 0, 0);

  EXPECT_CALL(
    *frame_manager_,
    transform("/map", _, _, _, _))  // NOLINT
  .WillRepeatedly(
    // NOLINT
    DoAll(
      SetArgReferee<3>(position), SetArgReferee<4>(orientation), Return(true)));

  auto message = createMapMessage();
  message->header.frame_id = "";
  map_display_->processMessage(message);

  auto manual_objects = rviz_default_plugins::findAllOgreObjectByType<Ogre::ManualObject>(
    scene_manager_->getRootSceneNode(), "ManualObject");

  ASSERT_THAT(manual_objects, SizeIs(1));
  EXPECT_TRUE(manual_objects[0]->isVisible());
  auto map_display_scene_node = manual_objects[0]->getParentSceneNode()->getParentSceneNode();
  EXPECT_THAT(map_display_scene_node->getPosition(), Vector3Eq(position));
  EXPECT_THAT(map_display_scene_node->getOrientation(), QuaternionEq(orientation));
}

TEST_F(MapTestFixture, showMap_does_not_display_anything_if_transform_fails) {
  map_display_->processMessage(createMapMessage());

  auto manual_objects = rviz_default_plugins::findAllOgreObjectByType<Ogre::ManualObject>(
    scene_manager_->getRootSceneNode(), "ManualObject");

  ASSERT_THAT(manual_objects, SizeIs(1));
  EXPECT_FALSE(manual_objects[0]->isVisible());
}

TEST_F(MapTestFixture, reset_deletes_map) {
  mockValidTransform();
  Ogre::Vector3 position(1, 0, 0);
  Ogre::Quaternion orientation(0.5f, 0.5f, 0, 0);

  map_display_->processMessage(createMapMessage());

  map_display_->reset();

  auto manual_objects = rviz_default_plugins::findAllOgreObjectByType<Ogre::ManualObject>(
    scene_manager_->getRootSceneNode(), "ManualObject");

  ASSERT_THAT(manual_objects, SizeIs(0));
}

TEST_F(MapTestFixture, createSwatches_creates_more_swatches_if_map_is_too_big) {
  // one dimension is larger than 2^16 --> that's too much for one texture buffer
  map_display_->processMessage(createMapMessage(70000, 50));

  auto manual_objects = rviz_default_plugins::findAllOgreObjectByType<Ogre::ManualObject>(
    scene_manager_->getRootSceneNode(), "ManualObject");

  EXPECT_THAT(manual_objects, SizeIs(2));
}
