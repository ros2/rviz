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

#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include "rviz_common/display.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_rendering/custom_parameter_indices.hpp"

#include "rviz_default_plugins/displays/pointcloud/point_cloud_common.hpp"

#include "../../scene_graph_introspection.hpp"
#include "../../pointcloud_messages.hpp"
#include "../display_test_fixture.hpp"

using namespace rviz_default_plugins;  // NOLINT
using namespace ::testing;  // NOLINT

class PointCloudCommonTestFixture : public DisplayTestFixture
{
public:
  void SetUp() override
  {
    DisplayTestFixture::SetUp();

    parent_display_ = std::make_shared<rviz_common::Display>();
    point_cloud_common_ = std::make_shared<rviz_default_plugins::PointCloudCommon>(
      parent_display_.get());
  }

  void TearDown() override
  {
    point_cloud_common_.reset();
    DisplayTestFixture::TearDown();
  }

  std::shared_ptr<rviz_common::Display> parent_display_;
  std::shared_ptr<rviz_default_plugins::PointCloudCommon> point_cloud_common_;
};

TEST_F(PointCloudCommonTestFixture, update_adds_pointcloud_to_scene_graph) {
  point_cloud_common_->initialize(
    context_.get(), scene_manager_->getRootSceneNode()->createChildSceneNode());

  // just plain Point is ambiguous on macOS
  rviz_default_plugins::Point p1 = {1, 2, 3};
  rviz_default_plugins::Point p2 = {4, 5, 6};
  auto cloud = createPointCloud2WithPoints(std::vector<rviz_default_plugins::Point>{p1, p2});

  mockValidTransform();

  point_cloud_common_->addMessage(cloud);
  point_cloud_common_->update(0, 0);

  auto point_cloud = rviz_default_plugins::findOnePointCloud(scene_manager_->getRootSceneNode());

  EXPECT_THAT(point_cloud->getPoints()[0].position, Vector3Eq(Ogre::Vector3(1, 2, 3)));
  EXPECT_THAT(point_cloud->getPoints()[1].position, Vector3Eq(Ogre::Vector3(4, 5, 6)));
}

TEST_F(PointCloudCommonTestFixture, update_removes_old_point_clouds) {
  point_cloud_common_->initialize(
    context_.get(), scene_manager_->getRootSceneNode()->createChildSceneNode());

  mockValidTransform();

  // just plain Point is ambiguous on macOS
  rviz_default_plugins::Point p = {1, 2, 3};
  auto cloud = createPointCloud2WithPoints(std::vector<rviz_default_plugins::Point>{p});

  point_cloud_common_->addMessage(cloud);
  point_cloud_common_->update(0, 0);

  p = {4, 5, 6};
  cloud = createPointCloud2WithPoints(std::vector<rviz_default_plugins::Point>{p});

  point_cloud_common_->addMessage(cloud);
  point_cloud_common_->update(0, 0);

  auto point_clouds = rviz_default_plugins::findAllPointClouds(scene_manager_->getRootSceneNode());
  ASSERT_THAT(point_clouds.size(), Eq(1u));

  EXPECT_THAT(point_clouds[0]->getPoints()[0].position, Vector3Eq(Ogre::Vector3(4, 5, 6)));
}

TEST_F(PointCloudCommonTestFixture, update_sets_size_and_alpha_on_renderable) {
  point_cloud_common_->initialize(
    context_.get(), scene_manager_->getRootSceneNode()->createChildSceneNode());

  // just plain Point is ambiguous on macOS
  rviz_default_plugins::Point p1 = {1, 2, 3};
  rviz_default_plugins::Point p2 = {4, 5, 6};
  auto cloud = createPointCloud2WithPoints(std::vector<rviz_default_plugins::Point>{p1, p2});

  mockValidTransform();

  point_cloud_common_->addMessage(cloud);
  point_cloud_common_->update(0, 0);

  auto point_cloud = rviz_default_plugins::findOnePointCloud(scene_manager_->getRootSceneNode());
  auto size = point_cloud->getRenderables()[0]->getCustomParameter(RVIZ_RENDERING_SIZE_PARAMETER);
  auto alpha =
    point_cloud->getRenderables()[0]->getCustomParameter(RVIZ_RENDERING_ALPHA_PARAMETER);

  EXPECT_THAT(Ogre::Vector3(size.x, size.y, size.z), Vector3Eq(Ogre::Vector3(0.01f, 0.01f, 0.01f)));
  EXPECT_THAT(Ogre::Vector3(alpha.x, alpha.y, alpha.z), Vector3Eq(Ogre::Vector3(1, 1, 1)));
}

TEST_F(PointCloudCommonTestFixture, update_adds_nothing_if_transform_fails) {
  point_cloud_common_->initialize(
    context_.get(), scene_manager_->getRootSceneNode()->createChildSceneNode());

  // just plain Point is ambiguous on macOS
  rviz_default_plugins::Point p1 = {1, 2, 3};
  rviz_default_plugins::Point p2 = {4, 5, 6};
  auto cloud = createPointCloud2WithPoints(std::vector<rviz_default_plugins::Point>{p1, p2});

  EXPECT_CALL(*frame_manager_, getTransform(_, _, _, _)).WillRepeatedly(Return(false));  // NOLINT

  point_cloud_common_->addMessage(cloud);
  point_cloud_common_->update(0, 0);

  auto point_cloud = rviz_default_plugins::findOnePointCloud(scene_manager_->getRootSceneNode());
  EXPECT_FALSE(point_cloud);
}

TEST_F(PointCloudCommonTestFixture, update_colors_the_points_using_the_selected_color_transformer) {
  point_cloud_common_->initialize(
    context_.get(), scene_manager_->getRootSceneNode()->createChildSceneNode());

  // just plain Point is ambiguous on macOS
  rviz_default_plugins::Point p1 = {1, 2, 3};
  rviz_default_plugins::Point p2 = {4, 5, 6};
  auto cloud = createPointCloud2WithPoints(std::vector<rviz_default_plugins::Point>{p1, p2});

  mockValidTransform();

  auto color_transformer_property = parent_display_->findProperty("Color Transformer");
  color_transformer_property->setValue("FlatColor");

  auto color_property = parent_display_->findProperty("Color");
  color_property->setValue(QColor(255, 0, 0));

  point_cloud_common_->addMessage(cloud);
  point_cloud_common_->update(0, 0);

  auto point_cloud = rviz_default_plugins::findOnePointCloud(scene_manager_->getRootSceneNode());

  EXPECT_THAT(point_cloud->getPoints()[0].position, Vector3Eq(Ogre::Vector3(1, 2, 3)));
  EXPECT_THAT(point_cloud->getPoints()[0].color, ColourValueEq(Ogre::ColourValue(1, 0, 0)));
}

TEST_F(
  PointCloudCommonTestFixture,
  sending_a_point_cloud_with_not_enough_data_results_in_error_but_no_crash)
{
  point_cloud_common_->initialize(
    context_.get(), scene_manager_->getRootSceneNode()->createChildSceneNode());

  mockValidTransform();

  auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
  cloud->header = std_msgs::msg::Header();
  cloud->header.stamp = rclcpp::Clock().now();

  cloud->is_bigendian = false;
  cloud->is_dense = true;

  cloud->height = 1;
  cloud->width = 100;

  cloud->fields.resize(3);
  cloud->fields[0].name = "x";
  cloud->fields[1].name = "y";
  cloud->fields[2].name = "z";

  sensor_msgs::msg::PointField::_offset_type offset = 0;
  for (uint32_t i = 0; i < cloud->fields.size(); ++i, offset += sizeof(float)) {
    cloud->fields[i].count = 1;
    cloud->fields[i].offset = offset;
    cloud->fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
  }

  cloud->point_step = offset;
  cloud->row_step = cloud->point_step * cloud->width;
  cloud->data.resize(cloud->row_step * 1 + 5);  // have the cloud be a weird size

  auto floatData = reinterpret_cast<float *>(cloud->data.data());
  for (uint32_t i = 0; i < 10; ++i) {  // we don't fill in enough data
    floatData[i * (cloud->point_step / sizeof(float)) + 0] = 1 + i;
    floatData[i * (cloud->point_step / sizeof(float)) + 1] = 2 + i;
    floatData[i * (cloud->point_step / sizeof(float)) + 2] = 3 + i;
  }

  point_cloud_common_->addMessage(cloud);
  point_cloud_common_->update(0, 0);

  auto point_clouds = rviz_default_plugins::findAllPointClouds(scene_manager_->getRootSceneNode());
  ASSERT_THAT(point_clouds.size(), Eq(0u));
}

int main(int argc, char ** argv)
{
  QApplication app(argc, argv);
  InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
