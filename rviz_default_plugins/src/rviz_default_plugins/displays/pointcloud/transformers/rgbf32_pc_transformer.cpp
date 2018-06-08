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

#include "rviz_default_plugins/displays/pointcloud/point_cloud_helpers.hpp"
#include "rviz_default_plugins/displays/pointcloud/transformers/rgbf32_pc_transformer.hpp"

namespace rviz_default_plugins
{

unsigned char RGBF32PCTransformer::supports(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud)
{
  int ri = findChannelIndex(cloud, "r");
  int gi = findChannelIndex(cloud, "g");
  int bi = findChannelIndex(cloud, "b");
  if (ri == -1 || gi == -1 || bi == -1) {
    return PointCloudTransformer::Support_None;
  }

  if (cloud->fields[ri].datatype == sensor_msgs::msg::PointField::FLOAT32) {
    return Support_Color;
  }

  return Support_None;
}

bool RGBF32PCTransformer::transform(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud,
  uint32_t mask,
  const Ogre::Matrix4 & transform,
  V_PointCloudPoint & points_out)
{
  (void) transform;

  if (!(mask & Support_Color)) {
    return false;
  }

  int32_t ri = findChannelIndex(cloud, "r");
  int32_t gi = findChannelIndex(cloud, "g");
  int32_t bi = findChannelIndex(cloud, "b");

  const uint32_t roff = cloud->fields[ri].offset;
  const uint32_t goff = cloud->fields[gi].offset;
  const uint32_t boff = cloud->fields[bi].offset;
  const uint32_t point_step = cloud->point_step;
  const uint32_t num_points = cloud->width * cloud->height;
  uint8_t const * point = &cloud->data.front();
  for (uint32_t i = 0; i < num_points; ++i, point += point_step) {
    float r = *reinterpret_cast<const float *>(point + roff);
    float g = *reinterpret_cast<const float *>(point + goff);
    float b = *reinterpret_cast<const float *>(point + boff);
    points_out[i].color = Ogre::ColourValue(r, g, b);
  }

  return true;
}

}  // end namespace rviz_default_plugins
