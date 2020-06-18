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

#include <OgreMaterialManager.h>

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "rviz_rendering/objects/point_cloud.hpp"
#include "rviz_common/interaction/selection_handler.hpp"

#include "../../../../src/rviz_default_plugins/displays/pointcloud/point_cloud_common.hpp"
#include "../../../../src/rviz_default_plugins/displays/pointcloud/point_cloud2_display.hpp"

#include "../../scene_graph_introspection.hpp"
#include "../display_test_fixture.hpp"
#include "./message_creators.hpp"

using namespace ::testing;  // NOLINT
using namespace rviz_default_plugins::displays;  // NOLINT

MATCHER_P(ContainsWireBoxWithBoundingBox, AABB, "") {
  for (const auto & box : arg) {
    if (box->getBoundingBox() == AABB) {
      return true;
    }
  }
  return false;
}

MATCHER_P(HasPositionPropertyWithPosition, position, "") {
  return arg->getNameStd() == "Position" &&
         arg->numChildren() == 3 &&
         abs(arg->childAt(0)->getValue().toFloat() - position.x) < 0.001 &&
         abs(arg->childAt(1)->getValue().toFloat() - position.y) < 0.001 &&
         abs(arg->childAt(2)->getValue().toFloat() - position.z) < 0.001;
}

MATCHER_P(HasNumberOfSubproperties, number, "") {
  return arg->numChildren() == number;
}

MATCHER_P2(HasColorProperty, color_string, alpha, "") {
  return arg->numChildren() == 3 &&
         arg->childAt(1)->getValue().toString().toStdString() == color_string &&
         abs(arg->childAt(2)->getValue().toFloat() - alpha) < 0.001;
}

MATCHER_P(HasIntensityProperty, intensity, "") {
  return arg->numChildren() == 2 && abs(arg->childAt(1)->getValue().toFloat() - intensity) < 0.001;
}

class PointCloudSelectionHandlerFixture : public DisplayTestFixture
{
public:
  PointCloudSelectionHandlerFixture()
  {
    Ogre::MaterialManager::getSingletonPtr()->load("RVIZ/Cyan", "rviz_rendering");
  }
};

std::shared_ptr<rviz_default_plugins::CloudInfo> createCloudInfoWithSquare(
  Ogre::SceneManager * scene_manager, rviz_common::DisplayContext * context,
  sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
{
  std::vector<rviz_rendering::PointCloud::Point> points;
  points.push_back({Ogre::Vector3(1, 1, 1), Ogre::ColourValue(0, 0, 0, 1)});
  points.push_back({Ogre::Vector3(-1, -1, 1), Ogre::ColourValue(0, 0, 0, 1)});
  points.push_back({Ogre::Vector3(-1, 1, 1), Ogre::ColourValue(0, 0, 0, 1)});
  points.push_back({Ogre::Vector3(1, -1, 1), Ogre::ColourValue(0, 0, 0, 1)});

  auto cloud = std::make_shared<rviz_rendering::PointCloud>();
  cloud->addPoints(points.begin(), points.end());

  auto cloud_info = std::make_shared<rviz_default_plugins::CloudInfo>();
  cloud_info->message_ = message;
  cloud_info->receive_time_ = message->header.stamp;
  cloud_info->manager_ = scene_manager;
  cloud_info->scene_node_ = scene_manager->getRootSceneNode()->createChildSceneNode();
  cloud_info->cloud_ = cloud;
  cloud_info->transformed_points_ = points;
  cloud_info->orientation_ = Ogre::Quaternion::IDENTITY;
  cloud_info->position_ = Ogre::Vector3::ZERO;
  cloud_info->setSelectable(true, 1, context);
  cloud_info->scene_node_ = scene_manager
    ->getRootSceneNode()
    ->createChildSceneNode()
    ->createChildSceneNode(cloud_info->position_, cloud_info->orientation_);
  return cloud_info;
}

TEST_F(PointCloudSelectionHandlerFixture, onSelect_selects_only_points_actually_picked)
{
  std::vector<rviz_default_plugins::Point> message_points =
  {{1, 1, 1}, {-1, -1, 1}, {-1, 1, 1}, {1, -1, 1}};
  auto message = rviz_default_plugins::createPointCloud2WithPoints(message_points);
  auto cloud_info = createCloudInfoWithSquare(scene_manager_, context_.get(), message);

  rviz_common::interaction::Picked picked_object(cloud_info->selection_handler_->getHandle());
  picked_object.extra_handles.clear();
  picked_object.extra_handles.insert(1);
  picked_object.extra_handles.insert(4);

  cloud_info->selection_handler_->onSelect(picked_object);
  rviz_common::interaction::V_AABB aabbs = cloud_info->selection_handler_->getAABBs(picked_object);

  EXPECT_THAT(aabbs, SizeIs(2));
  auto found_objects = rviz_default_plugins::findAllOgreObjectByType<Ogre::SimpleRenderable>(
    scene_manager_->getRootSceneNode(), "SimpleRenderable");
  EXPECT_THAT(
    found_objects,
    ContainsWireBoxWithBoundingBox(Ogre::AxisAlignedBox(0.5f, 0.5f, 0.5f, 1.5f, 1.5f, 1.5f)));
  EXPECT_THAT(
    found_objects,
    ContainsWireBoxWithBoundingBox(Ogre::AxisAlignedBox(0.5f, -1.5f, 0.5f, 1.5f, -0.5f, 1.5f)));
}

TEST_F(
  PointCloudSelectionHandlerFixture,
  onDeselect_destroys_wired_bounding_boxes_for_unpicked_objects)
{
  std::vector<rviz_default_plugins::Point> message_points =
  {{1, 1, 1}, {-1, -1, 1}, {-1, 1, 1}, {1, -1, 1}};
  auto message = rviz_default_plugins::createPointCloud2WithPoints(message_points);
  auto cloud_info = createCloudInfoWithSquare(scene_manager_, context_.get(), message);

  rviz_common::interaction::Picked picked_object(cloud_info->selection_handler_->getHandle());
  picked_object.extra_handles.clear();
  picked_object.extra_handles.insert(1);
  picked_object.extra_handles.insert(4);
  cloud_info->selection_handler_->onSelect(picked_object);

  rviz_common::interaction::Picked unpicked_object(cloud_info->selection_handler_->getHandle());
  unpicked_object.extra_handles.clear();
  unpicked_object.extra_handles.insert(1);
  cloud_info->selection_handler_->onDeselect(unpicked_object);

  rviz_common::interaction::V_AABB aabbs = cloud_info->selection_handler_->getAABBs(picked_object);

  EXPECT_THAT(aabbs, SizeIs(1));
  auto found_objects = rviz_default_plugins::findAllOgreObjectByType<Ogre::SimpleRenderable>(
    scene_manager_->getRootSceneNode(), "SimpleRenderable");
  EXPECT_THAT(
    found_objects,
    ContainsWireBoxWithBoundingBox(Ogre::AxisAlignedBox(0.5f, -1.5f, 0.5f, 1.5f, -0.5f, 1.5f)));
}

TEST_F(
  PointCloudSelectionHandlerFixture,
  createProperties_creates_tree_of_properties_without_color_for_simple_clouds)
{
  std::vector<rviz_default_plugins::Point> message_points =
  {{1, 1, 1}, {-1, -1, 1}, {-1, 1, 1}, {1, -1, 1}};
  auto message = rviz_default_plugins::createPointCloud2WithPoints(message_points);
  auto cloud_info = createCloudInfoWithSquare(scene_manager_, context_.get(), message);

  rviz_common::interaction::Picked picked_object(cloud_info->selection_handler_->getHandle());
  picked_object.extra_handles.clear();
  picked_object.extra_handles.insert(1);
  picked_object.extra_handles.insert(4);
  cloud_info->selection_handler_->onSelect(picked_object);

  rviz_common::properties::Property * parent = new rviz_common::properties::Property();
  cloud_info->selection_handler_->createProperties(picked_object, parent);

  ASSERT_THAT(parent, HasNumberOfSubproperties(2));

  auto first_point_property = parent->childAt(0);
  ASSERT_THAT(first_point_property, HasNumberOfSubproperties(1));
  EXPECT_THAT(first_point_property->getNameStd(), StartsWith("Point 0 [cloud"));
  EXPECT_THAT(
    first_point_property->childAt(0),
    HasPositionPropertyWithPosition(Ogre::Vector3(1, 1, 1)));

  auto second_point_property = parent->childAt(1);
  ASSERT_THAT(second_point_property, HasNumberOfSubproperties(1));
  EXPECT_THAT(second_point_property->getNameStd(), StartsWith("Point 3 [cloud"));
  EXPECT_THAT(
    second_point_property->childAt(0),
    HasPositionPropertyWithPosition(Ogre::Vector3(1, -1, 1)));
}

TEST_F(
  PointCloudSelectionHandlerFixture,
  createProperties_creates_tree_of_properties_with_color_for_clouds_with_rgb)
{
  std::vector<rviz_default_plugins::ColoredPoint> message_points =
  {{1, 1, 1, 0, 0, 0},
    {-1, -1, 1, 50, 50, 50},
    {-1, 1, 1, 100, 100, 100},
    {1, -1, 1, 255, 255, 255}};
  auto message = rviz_default_plugins::create8BitColoredPointCloud2(message_points);
  auto cloud_info = createCloudInfoWithSquare(scene_manager_, context_.get(), message);

  rviz_common::interaction::Picked picked_object(cloud_info->selection_handler_->getHandle());
  picked_object.extra_handles.clear();
  picked_object.extra_handles.insert(1);
  picked_object.extra_handles.insert(4);
  cloud_info->selection_handler_->onSelect(picked_object);

  rviz_common::properties::Property * parent = new rviz_common::properties::Property();
  cloud_info->selection_handler_->createProperties(picked_object, parent);

  ASSERT_THAT(parent, HasNumberOfSubproperties(2));
  EXPECT_THAT(parent->childAt(0), HasColorProperty("0; 0; 0", 0.0f));
  EXPECT_THAT(parent->childAt(1), HasColorProperty("1; 1; 1", 0.0f));
}

TEST_F(
  PointCloudSelectionHandlerFixture,
  createProperties_creates_tree_of_properties_with_intensity_for_clouds_with_intensity)
{
  std::vector<rviz_default_plugins::PointWithIntensity> message_points =
  {{1, 1, 1, 0}, {-1, -1, 1, 0.5f}, {-1, 1, 1, 0.5f}, {1, -1, 1, 1}};
  auto message = rviz_default_plugins::createPointCloud2WithIntensity(message_points);
  Ogre::MaterialManager::getSingletonPtr()->load("RVIZ/Cyan", "rviz_rendering");
  auto cloud_info = createCloudInfoWithSquare(scene_manager_, context_.get(), message);

  rviz_common::interaction::Picked picked_object(cloud_info->selection_handler_->getHandle());
  picked_object.extra_handles.clear();
  picked_object.extra_handles.insert(1);
  picked_object.extra_handles.insert(4);
  cloud_info->selection_handler_->onSelect(picked_object);

  rviz_common::properties::Property * parent = new rviz_common::properties::Property();
  cloud_info->selection_handler_->createProperties(picked_object, parent);

  ASSERT_THAT(parent, HasNumberOfSubproperties(2));
  ASSERT_THAT(parent->childAt(0), HasIntensityProperty(0.0f));
  ASSERT_THAT(parent->childAt(1), HasIntensityProperty(1.0f));
}
