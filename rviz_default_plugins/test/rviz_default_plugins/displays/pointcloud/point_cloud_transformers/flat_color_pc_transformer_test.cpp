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

#include <vector>

#include <QList>  // NOLINT: cpplint is unable to handle the include order here

#include "rviz_common/properties/property.hpp"
#include "../../../pointcloud_messages.hpp"

#include "rviz_default_plugins/displays/pointcloud/transformers/flat_color_pc_transformer.hpp"

using namespace ::testing;  // NOLINT
using namespace rviz_default_plugins;  // NOLINT

TEST(FlatColorPCTransformer, transform_colors_all_points_white_per_default) {
  // just plain Point is ambiguous on macOS
  rviz_default_plugins::Point p1 = {0, 0, 0};
  rviz_default_plugins::Point p2 = {0, 0, 1};
  rviz_default_plugins::Point p3 = {0, 0, 2};
  auto cloud = createPointCloud2WithPoints(std::vector<rviz_default_plugins::Point>{p1, p2, p3});

  V_PointCloudPoint points_out;
  points_out.resize(3);

  QList<rviz_common::properties::Property *> out_props;

  FlatColorPCTransformer transformer;
  transformer.createProperties(nullptr, PointCloudTransformer::Support_Color, out_props);

  ASSERT_THAT(out_props[0]->getNameStd(), StrEq("Color"));
  out_props[0]->setValue(QColor(Qt::red));

  transformer.transform(
    cloud, PointCloudTransformer::Support_Color, Ogre::Matrix4::IDENTITY, points_out);

  ASSERT_THAT(points_out[0].color, Eq(Ogre::ColourValue(1, 0, 0)));
  ASSERT_THAT(points_out[1].color, Eq(Ogre::ColourValue(1, 0, 0)));
  ASSERT_THAT(points_out[2].color, Eq(Ogre::ColourValue(1, 0, 0)));
}


TEST(FlatColorPCTransformer, transform_colors_all_points_in_the_given_color) {
  // just plain Point is ambiguous on macOS
  rviz_default_plugins::Point p1 = {0, 0, 0};
  rviz_default_plugins::Point p2 = {0, 0, 1};
  rviz_default_plugins::Point p3 = {0, 0, 2};
  auto cloud = createPointCloud2WithPoints(std::vector<rviz_default_plugins::Point>{p1, p2, p3});

  V_PointCloudPoint points_out;
  points_out.resize(3);

  QList<rviz_common::properties::Property *> out_props;

  FlatColorPCTransformer transformer;
  transformer.createProperties(nullptr, PointCloudTransformer::Support_Color, out_props);
  transformer.transform(
    cloud, PointCloudTransformer::Support_Color, Ogre::Matrix4::IDENTITY, points_out);

  ASSERT_THAT(points_out[0].color, Eq(Ogre::ColourValue(1, 1, 1)));
  ASSERT_THAT(points_out[1].color, Eq(Ogre::ColourValue(1, 1, 1)));
  ASSERT_THAT(points_out[2].color, Eq(Ogre::ColourValue(1, 1, 1)));
}
