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

#include "../../../pointcloud_messages.hpp"

#include "rviz_default_plugins/displays/pointcloud/transformers/intensity_pc_transformer.hpp"

using namespace ::testing;  // NOLINT
using namespace rviz_default_plugins;  // NOLINT

TEST(IntensityPCTransformer, transform_returns_points_colored_depending_on_the_intensity) {
  PointWithIntensity p1 = {0, 0, 0, 0};
  PointWithIntensity p2 = {0, 0, 0, 1};
  PointWithIntensity p3 = {0, 0, 0, 2};
  auto cloud = createPointCloud2WithIntensity(std::vector<PointWithIntensity>{p1, p2, p3});

  V_PointCloudPoint points_out;
  points_out.resize(3);

  QList<rviz_common::properties::Property *> out_props;

  IntensityPCTransformer transformer;
  transformer.createProperties(nullptr, PointCloudTransformer::Support_Color, out_props);

  transformer.transform(
    cloud, PointCloudTransformer::Support_Color, Ogre::Matrix4::IDENTITY, points_out);

  ASSERT_THAT(points_out[0].color, Eq(Ogre::ColourValue(1, 0, 0)));  // 0
  ASSERT_THAT(points_out[1].color, Eq(Ogre::ColourValue(0, 1, 0.5)));  // 1/2
  ASSERT_THAT(points_out[2].color, Eq(Ogre::ColourValue(1, 0, 1)));  // 1
}

TEST(
  IntensityPCTransformer,
  transform_interpolates_between_min_and_max_color_if_use_rainbow_is_diabled) {
  PointWithIntensity p1 = {0, 0, 0, 0};
  PointWithIntensity p2 = {0, 0, 0, 1};
  PointWithIntensity p3 = {0, 0, 0, 2};
  auto cloud = createPointCloud2WithIntensity(std::vector<PointWithIntensity>{p1, p2, p3});

  V_PointCloudPoint points_out;
  points_out.resize(3);

  QList<rviz_common::properties::Property *> out_props;

  IntensityPCTransformer transformer;
  transformer.createProperties(nullptr, PointCloudTransformer::Support_Color, out_props);

  ASSERT_THAT(out_props[1]->getNameStd(), StrEq("Use rainbow"));
  out_props[1]->setValue(false);

  ASSERT_THAT(out_props[3]->getNameStd(), StrEq("Min Color"));
  out_props[3]->setValue(QColor(0, 0, 0));

  ASSERT_THAT(out_props[4]->getNameStd(), StrEq("Max Color"));
  out_props[4]->setValue(QColor(255, 0, 0));

  transformer.transform(
    cloud, PointCloudTransformer::Support_Color, Ogre::Matrix4::IDENTITY, points_out);

  ASSERT_THAT(points_out[0].color, Eq(Ogre::ColourValue(0, 0, 0)));  // 0
  ASSERT_THAT(points_out[1].color, Eq(Ogre::ColourValue(0.5, 0, 0)));  // 1/2
  ASSERT_THAT(points_out[2].color, Eq(Ogre::ColourValue(1, 0, 0)));  // 1
}

TEST(
  IntensityPCTransformer,
  transform_uses_default_min_max_intensity_if_autocompute_bounds_is_disabled) {
  PointWithIntensity p1 = {0, 0, 0, 0};
  PointWithIntensity p2 = {0, 0, 0, 1024};
  PointWithIntensity p3 = {0, 0, 0, 2048};
  auto cloud = createPointCloud2WithIntensity(std::vector<PointWithIntensity>{p1, p2, p3});

  V_PointCloudPoint points_out;
  points_out.resize(3);

  QList<rviz_common::properties::Property *> out_props;

  IntensityPCTransformer transformer;
  transformer.createProperties(nullptr, PointCloudTransformer::Support_Color, out_props);

  ASSERT_THAT(out_props[5]->getNameStd(), StrEq("Autocompute Intensity Bounds"));
  out_props[5]->setValue(false);

  transformer.transform(
    cloud, PointCloudTransformer::Support_Color, Ogre::Matrix4::IDENTITY, points_out);

  ASSERT_THAT(points_out[0].color, Eq(Ogre::ColourValue(1, 0, 0)));  // 0
  ASSERT_THAT(points_out[1].color, Eq(Ogre::ColourValue(0.75, 1, 0)));  // 1/4
  ASSERT_THAT(points_out[2].color, Eq(Ogre::ColourValue(0, 1, 0.5)));  // 1/2
}
