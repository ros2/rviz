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

#include <algorithm>

#include "rviz_default_plugins/displays/pointcloud/point_cloud_helpers.hpp"

#include "rviz_default_plugins/displays/pointcloud/transformers/rgb8_pc_transformer.hpp"

namespace rviz_default_plugins
{

uint8_t RGB8PCTransformer::supports(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud)
{
  int32_t index = std::max(findChannelIndex(cloud, "rgb"), findChannelIndex(cloud, "rgba"));
  if (index == -1) {
    return Support_None;
  }

  if (cloud->fields[index].datatype == sensor_msgs::msg::PointField::INT32 ||
    cloud->fields[index].datatype == sensor_msgs::msg::PointField::UINT32 ||
    cloud->fields[index].datatype == sensor_msgs::msg::PointField::FLOAT32)
  {
    return Support_Color;
  }

  return Support_None;
}

bool RGB8PCTransformer::transform(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud,
  uint32_t mask,
  const Ogre::Matrix4 & transform,
  V_PointCloudPoint & points_out)
{
  (void) transform;

  if (!(mask & Support_Color)) {
    return false;
  }

  const int32_t rgb = findChannelIndex(cloud, "rgb");
  const int32_t rgba = findChannelIndex(cloud, "rgba");
  const int32_t index = std::max(rgb, rgba);

  const uint32_t off = cloud->fields[index].offset;
  uint8_t const * rgb_ptr = &cloud->data.front() + off;
  const uint32_t point_step = cloud->point_step;

  // Create a look-up table for colors
  float rgb_lut[256];
  for (int i = 0; i < 256; ++i) {
    rgb_lut[i] = static_cast<float>(i) / 255.0f;
  }
  if (rgb != -1) {  // rgb
    for (V_PointCloudPoint::iterator iter = points_out.begin(); iter != points_out.end();
      ++iter, rgb_ptr += point_step)
    {
      uint32_t rgb = *reinterpret_cast<const uint32_t *>(rgb_ptr);
      iter->color.r = rgb_lut[(rgb >> 16) & 0xff];
      iter->color.g = rgb_lut[(rgb >> 8) & 0xff];
      iter->color.b = rgb_lut[rgb & 0xff];
      iter->color.a = 1.0f;
    }
  } else {  // rgba
    for (V_PointCloudPoint::iterator iter = points_out.begin(); iter != points_out.end();
      ++iter, rgb_ptr += point_step)
    {
      uint32_t rgb = *reinterpret_cast<const uint32_t *>(rgb_ptr);
      iter->color.r = rgb_lut[(rgb >> 16) & 0xff];
      iter->color.g = rgb_lut[(rgb >> 8) & 0xff];
      iter->color.b = rgb_lut[rgb & 0xff];
      iter->color.a = rgb_lut[rgb >> 24];
    }
  }

  return true;
}

}  // end namespace rviz_default_plugins
