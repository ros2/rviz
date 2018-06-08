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
#include "rviz_default_plugins/displays/pointcloud/transformers/xyz_pc_transformer.hpp"

namespace rviz_default_plugins
{

uint8_t XYZPCTransformer::supports(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud)
{
  int32_t xi = rviz_default_plugins::findChannelIndex(cloud, "x");
  int32_t yi = rviz_default_plugins::findChannelIndex(cloud, "y");
  int32_t zi = rviz_default_plugins::findChannelIndex(cloud, "z");

  if (xi == -1 || yi == -1 || zi == -1) {
    return PointCloudTransformer::Support_None;
  }

  if (cloud->fields[xi].datatype == sensor_msgs::msg::PointField::FLOAT32) {
    return PointCloudTransformer::Support_XYZ;
  }

  return rviz_default_plugins::PointCloudTransformer::Support_None;
}

bool XYZPCTransformer::transform(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud,
  uint32_t mask,
  const Ogre::Matrix4 & transform,
  V_PointCloudPoint & points_out)
{
  (void) transform;
  if (!(mask & PointCloudTransformer::Support_XYZ)) {
    return false;
  }

  int32_t xi = findChannelIndex(cloud, "x");
  int32_t yi = findChannelIndex(cloud, "y");
  int32_t zi = findChannelIndex(cloud, "z");

  const uint32_t xoff = cloud->fields[xi].offset;
  const uint32_t yoff = cloud->fields[yi].offset;
  const uint32_t zoff = cloud->fields[zi].offset;
  const uint32_t point_step = cloud->point_step;
  uint8_t const * point_x = &cloud->data.front() + xoff;
  uint8_t const * point_y = &cloud->data.front() + yoff;
  uint8_t const * point_z = &cloud->data.front() + zoff;
  for (V_PointCloudPoint::iterator iter = points_out.begin(); iter != points_out.end();
    ++iter, point_x += point_step,
    point_y += point_step, point_z += point_step)
  {
    iter->position.x = *reinterpret_cast<const float *>(point_x);
    iter->position.y = *reinterpret_cast<const float *>(point_y);
    iter->position.z = *reinterpret_cast<const float *>(point_z);
  }

  return true;
}

}  // end namespace rviz_default_plugins
