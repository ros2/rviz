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

#include "rviz_default_plugins/displays/pointcloud/transformers/axis_color_pc_transformer.hpp"

using namespace ::testing;  // NOLINT
using namespace rviz_default_plugins;  // NOLINT

TEST(AxisColorPCTransformer, transform_returns_points_colored_depending_on_the_z_position) {
  // just plain Point is ambiguous on macOS
  rviz_default_plugins::Point p1 = {0, 0, 0};
  rviz_default_plugins::Point p2 = {0, 0, 1};
  rviz_default_plugins::Point p3 = {0, 0, 2};
  auto cloud = createPointCloud2WithPoints(std::vector<rviz_default_plugins::Point>{p1, p2, p3});

  V_PointCloudPoint points_out;
  points_out.resize(3);

  QList<rviz_common::properties::Property *> out_props;

  AxisColorPCTransformer transformer;
  transformer.createProperties(nullptr, PointCloudTransformer::Support_Color, out_props);
  transformer.transform(
    cloud, PointCloudTransformer::Support_Color, Ogre::Matrix4::IDENTITY, points_out);

  ASSERT_THAT(points_out[0].color, Eq(Ogre::ColourValue(1, 0, 0)));
  ASSERT_THAT(points_out[1].color, Eq(Ogre::ColourValue(0, 1, 0.5)));
  ASSERT_THAT(points_out[2].color, Eq(Ogre::ColourValue(1, 0, 1)));
}

TEST(AxisColorPCTransformer, transform_uses_default_min_max_if_autocomplete_value_bounds_is_false) {
  // just plain Point is ambiguous on macOS
  rviz_default_plugins::Point p1 = {0, 0, 0};
  rviz_default_plugins::Point p2 = {0, 0, 1};
  rviz_default_plugins::Point p3 = {0, 0, 2};
  auto cloud = createPointCloud2WithPoints(std::vector<rviz_default_plugins::Point>{p1, p2, p3});

  V_PointCloudPoint points_out;
  points_out.resize(3);

  QList<rviz_common::properties::Property *> out_props;

  AxisColorPCTransformer transformer;
  transformer.createProperties(nullptr, PointCloudTransformer::Support_Color, out_props);

  ASSERT_THAT(out_props[1]->getNameStd(), StrEq("Autocompute Value Bounds"));
  out_props[1]->setValue(QVariant(false));

  transformer.transform(
    cloud, PointCloudTransformer::Support_Color, Ogre::Matrix4::IDENTITY, points_out);

  ASSERT_THAT(points_out[0].color, Eq(Ogre::ColourValue(0, 1, 0.5)));
  ASSERT_THAT(points_out[1].color, Eq(Ogre::ColourValue(0, 1, 0.75)));
  ASSERT_THAT(points_out[2].color, Eq(Ogre::ColourValue(0, 1, 1)));
}

TEST(AxisColorPCTransformer, transform_should_not_transform_points_when_using_local_frame) {
  // just plain Point is ambiguous on macOS
  rviz_default_plugins::Point p1 = {0, 0, 0};
  rviz_default_plugins::Point p2 = {0, 0, 1};
  rviz_default_plugins::Point p3 = {0, 0, 2};
  auto cloud = createPointCloud2WithPoints(std::vector<rviz_default_plugins::Point>{p1, p2, p3});

  V_PointCloudPoint points_out;
  points_out.resize(3);

  QList<rviz_common::properties::Property *> out_props;

  AxisColorPCTransformer transformer;
  transformer.createProperties(nullptr, PointCloudTransformer::Support_Color, out_props);
  ASSERT_THAT(out_props[2]->getNameStd(), StrEq("Use Fixed Frame"));

  out_props[2]->setValue(false);

  // This matrix would fail the transformation if fixed frame = true
  Ogre::Matrix4 tranformation(Ogre::Matrix4::ZERO);

  transformer.transform(
    cloud, PointCloudTransformer::Support_Color, tranformation, points_out);

  ASSERT_THAT(points_out[0].color, Eq(Ogre::ColourValue(1, 0, 0)));
  ASSERT_THAT(points_out[1].color, Eq(Ogre::ColourValue(0, 1, 0.5)));
  ASSERT_THAT(points_out[2].color, Eq(Ogre::ColourValue(1, 0, 1)));
}
