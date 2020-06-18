/*
 * Copyright (c) 2017, Bosch Software Innovations GmbH.
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
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
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
#include <vector>

#include "../../../pointcloud_messages.hpp"

#include "../../../scene_graph_introspection.hpp"

#include "rviz_default_plugins/displays/pointcloud/transformers/xyz_pc_transformer.hpp"

using namespace ::testing;  // NOLINT
using namespace rviz_default_plugins;  // NOLINT

TEST(XYZPCTransformer, transform_returns_the_point_cloud_points) {
  // just plain Point is ambiguous on macOS
  rviz_default_plugins::Point p1 = {1, 2, 3};
  rviz_default_plugins::Point p2 = {4, 5, 6};
  auto cloud = createPointCloud2WithPoints(std::vector<rviz_default_plugins::Point>{p1, p2});

  V_PointCloudPoint points_out;
  points_out.resize(2);

  XYZPCTransformer transformer;
  transformer.transform(cloud, PointCloudTransformer::Support_XYZ, Ogre::Matrix4::ZERO, points_out);

  ASSERT_THAT(points_out, SizeIs(2));
  ASSERT_THAT(points_out[0].position, Vector3Eq(Ogre::Vector3(1, 2, 3)));
  ASSERT_THAT(points_out[1].position, Vector3Eq(Ogre::Vector3(4, 5, 6)));
}

TEST(XYZPCTransformer, transform_returns_false_if_cloud_doesnt_support_xyz) {
  auto cloud = createPointCloud2WithSquare();

  V_PointCloudPoint points_out;

  XYZPCTransformer transformer;
  bool result = transformer.transform(
    cloud, PointCloudTransformer::Support_None, Ogre::Matrix4::ZERO, points_out);

  ASSERT_FALSE(result);
}

TEST(XYZPCTransformer, supports_returns_xyz_support_for_cloud_with_xyz_fields) {
  auto cloud = createPointCloud2WithSquare();

  XYZPCTransformer transformer;
  uint8_t result = transformer.supports(cloud);

  ASSERT_THAT(result, Eq(PointCloudTransformer::Support_XYZ));
}

TEST(XYZPCTransformer, supports_returns_no_xyz_support_for_cloud_without_xyz_fields) {
  auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();

  cloud->fields.resize(3);
  cloud->fields[0].name = "a";
  cloud->fields[1].name = "b";
  cloud->fields[2].name = "c";

  XYZPCTransformer transformer;
  uint8_t result = transformer.supports(cloud);

  ASSERT_THAT(result, Eq(PointCloudTransformer::Support_None));
}
