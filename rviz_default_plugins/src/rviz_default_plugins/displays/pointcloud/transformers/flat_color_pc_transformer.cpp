/*
 * Copyright (c) 2010, Willow Garage, Inc.
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

#include "rviz_common/properties/color_property.hpp"
#include "rviz_default_plugins/displays/pointcloud/point_cloud_helpers.hpp"

#include "rviz_default_plugins/displays/pointcloud/transformers/flat_color_pc_transformer.hpp"

namespace rviz_default_plugins
{

uint8_t FlatColorPCTransformer::supports(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud)
{
  (void) cloud;
  return PointCloudTransformer::Support_Color;
}

uint8_t FlatColorPCTransformer::score(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud)
{
  (void) cloud;
  return 0;
}

bool FlatColorPCTransformer::transform(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud,
  uint32_t mask,
  const Ogre::Matrix4 & transform,
  V_PointCloudPoint & points_out)
{
  (void) transform;
  if (!(mask & PointCloudTransformer::Support_Color)) {
    return false;
  }

  Ogre::ColourValue color = color_property_->getOgreColor();

  const uint32_t num_points = cloud->width * cloud->height;
  for (uint32_t i = 0; i < num_points; ++i) {
    points_out[i].color = color;
  }

  return true;
}

void FlatColorPCTransformer::createProperties(
  rviz_common::properties::Property * parent_property,
  uint32_t mask,
  QList<rviz_common::properties::Property *> & out_props)
{
  if (mask & PointCloudTransformer::Support_Color) {
    color_property_ = new rviz_common::properties::ColorProperty(
      "Color", Qt::white,
      "Color to assign to every point.",
      parent_property, SIGNAL(needRetransform()),
      this);
    out_props.push_back(color_property_);
  }
}

}  // end namespace rviz_default_plugins
