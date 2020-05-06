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

#include "rviz_default_plugins/displays/pointcloud/transformers/rgb8_pc_transformer.hpp"

using namespace ::testing;  // NOLINT
using namespace rviz_default_plugins; // NOLINT

TEST(RGB8PCTransformer, transform_returns_points_colored_in_their_rgb_color) {
  ColoredPoint p1 = {0, 0, 0, 0, 1, 0};
  ColoredPoint p2 = {0, 0, 0, 1, 0, 1};
  auto cloud = create8BitColoredPointCloud2(std::vector<ColoredPoint>{p1, p2});

  V_PointCloudPoint points_out;
  points_out.resize(2);

  RGB8PCTransformer transformer;
  transformer.transform(
    cloud, PointCloudTransformer::Support_Color, Ogre::Matrix4::ZERO, points_out);

  ASSERT_THAT(points_out, SizeIs(2));
  ASSERT_THAT(points_out[0].color, Eq(Ogre::ColourValue(0, 1, 0)));
  ASSERT_THAT(points_out[1].color, Eq(Ogre::ColourValue(1, 0, 1)));
}

TEST(RGBF32PCTransformer, supports_returns_color_support_for_cloud_with_rgb_field) {
  ColoredPoint p1 = {0, 0, 0, 0, 0, 0};
  ColoredPoint p2 = {0, 0, 0, 1, 1, 1};
  auto cloud = create8BitColoredPointCloud2(std::vector<ColoredPoint>{p1, p2});

  RGB8PCTransformer transformer;
  uint8_t result = transformer.supports(cloud);

  ASSERT_THAT(result, Eq(PointCloudTransformer::Support_Color));
}

TEST(RGBF32PCTransformer, supports_returns_no_color_support_for_cloud_without_rgb_field) {
  auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();

  cloud->fields.resize(3);
  cloud->fields[0].name = "a";
  cloud->fields[1].name = "b";
  cloud->fields[2].name = "c";

  RGB8PCTransformer transformer;
  uint8_t result = transformer.supports(cloud);

  ASSERT_THAT(result, Eq(PointCloudTransformer::Support_None));
}
