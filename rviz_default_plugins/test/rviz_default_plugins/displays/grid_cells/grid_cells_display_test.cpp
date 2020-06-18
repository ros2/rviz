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
#include <vector>

#include <OgreRoot.h>

#include "rviz_common/properties/float_property.hpp"

#include "rviz_default_plugins/displays/grid_cells/grid_cells_display.hpp"

#include "../../scene_graph_introspection.hpp"
#include "../display_test_fixture.hpp"

using namespace ::testing;  // NOLINT

class GridCellsDisplayFixture : public DisplayTestFixture
{
public:
  GridCellsDisplayFixture()
  {
    EXPECT_CALL(*context_, getFrameCount())
    .WillOnce(Return(0))
    .WillOnce(Return(0))
    .WillRepeatedly(Return(1));
    display_ = std::make_unique<rviz_default_plugins::displays::GridCellsDisplay>(context_.get());
    display_->setupCloud();
  }

  std::unique_ptr<rviz_default_plugins::displays::GridCellsDisplay> display_;
};

geometry_msgs::msg::Point point(float x, float y, float z)
{
  auto point = geometry_msgs::msg::Point();
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

nav_msgs::msg::GridCells::SharedPtr createGridCellsMessageWithTwoCells(
  float cell_width = 1, float cell_height = 1)
{
  auto message = std::make_shared<nav_msgs::msg::GridCells>();
  message->header = std_msgs::msg::Header();
  message->header.frame_id = "grid_cells_frame";
  message->header.stamp = rclcpp::Clock().now();

  message->cell_width = cell_width;
  message->cell_height = cell_height;

  std::vector<geometry_msgs::msg::Point> points;
  points.push_back(point(1, 1, 0));
  points.push_back(point(-1, -1, 0));

  message->cells = points;

  return message;
}

TEST_F(GridCellsDisplayFixture, processMessage_with_invalid_transform_returns_early) {
  EXPECT_CALL(*frame_manager_, getTransform(_, _, _, _)).WillOnce(Return(false));

  auto msg = createGridCellsMessageWithTwoCells();
  display_->processMessage(msg);

  auto point_clouds = rviz_default_plugins::findAllPointClouds(scene_manager_->getRootSceneNode());
  EXPECT_THAT(point_clouds.size(), Eq(1u));
  EXPECT_THAT(point_clouds[0]->getPoints().size(), Eq(0u));
}

TEST_F(GridCellsDisplayFixture, processMessage_with_zero_size_does_not_add_messages) {
  mockValidTransform();

  auto msg = createGridCellsMessageWithTwoCells(0, 1);
  display_->processMessage(msg);

  auto point_clouds = rviz_default_plugins::findAllPointClouds(scene_manager_->getRootSceneNode());
  EXPECT_THAT(point_clouds.size(), Eq(1u));
  EXPECT_THAT(point_clouds[0]->getPoints().size(), Eq(0u));
}

TEST_F(GridCellsDisplayFixture, processMessage_fills_pointcloud_with_correct_grid_cells_message) {
  mockValidTransform();
  auto msg = createGridCellsMessageWithTwoCells();
  display_->processMessage(msg);

  auto point_clouds = rviz_default_plugins::findAllPointClouds(scene_manager_->getRootSceneNode());
  EXPECT_THAT(point_clouds.size(), Eq(1u));
  EXPECT_THAT(point_clouds[0]->getPoints().size(), Eq(2u));
  EXPECT_THAT(
    point_clouds[0]->getParentSceneNode()->getPosition(), Vector3Eq(Ogre::Vector3(0, 1, 0)));
  EXPECT_THAT(
    point_clouds[0]->getParentSceneNode()->getOrientation(),
    QuaternionEq(Ogre::Quaternion(0, 0, 1, 0)));
}

TEST_F(GridCellsDisplayFixture, processMessage_clears_cloud_on_new_message) {
  mockValidTransform();

  auto msg = createGridCellsMessageWithTwoCells();
  display_->processMessage(msg);

  auto broken_msg = createGridCellsMessageWithTwoCells(0, 1);
  display_->processMessage(broken_msg);

  auto point_clouds = rviz_default_plugins::findAllPointClouds(scene_manager_->getRootSceneNode());
  EXPECT_THAT(point_clouds.size(), Eq(1u));
  EXPECT_THAT(point_clouds[0]->getPoints().size(), Eq(0u));
}
