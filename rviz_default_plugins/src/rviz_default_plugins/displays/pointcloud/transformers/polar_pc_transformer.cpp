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
#include "rviz_default_plugins/displays/pointcloud/transformers/polar_pc_transformer.hpp"
#include <math.h>

float DegreesToRadians(float degrees) {
    return degrees * (M_PI / 180);
}

namespace rviz_default_plugins
{

uint8_t PolarPCTransformer::supports(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud)
{
  int32_t rangei = findChannelIndex(cloud, "Range");
  int32_t azimuthi = findChannelIndex(cloud, "Azimuth");
  int32_t elevationi = findChannelIndex(cloud, "Elevation");

  if (rangei == -1 || azimuthi == -1 || elevationi == -1) {
    return PointCloudTransformer::Support_None;
  }

  if (cloud->fields[rangei].datatype == sensor_msgs::msg::PointField::FLOAT32) {
    return PointCloudTransformer::Support_XYZ;
  }

  return rviz_default_plugins::PointCloudTransformer::Support_None;
}

bool PolarPCTransformer::transform(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud,
  uint32_t mask,
  const Ogre::Matrix4 & transform,
  V_PointCloudPoint & points_out)
{
  (void) transform;
  if (!(mask & PointCloudTransformer::Support_XYZ)) {
    return false;
  }

  int32_t rangei = findChannelIndex(cloud, "Range");
  int32_t azimuthi = findChannelIndex(cloud, "Azimuth");
  int32_t elevationi = findChannelIndex(cloud, "Elevation");

  const uint32_t rangeoff = cloud->fields[rangei].offset;
  const uint32_t azimuthoff = cloud->fields[azimuthi].offset;
  const uint32_t elevationoff = cloud->fields[elevationi].offset;
  const uint32_t point_step = cloud->point_step;
  uint8_t const * point_range = &cloud->data.front() + rangeoff;
  uint8_t const * point_azimuth = &cloud->data.front() + azimuthoff;
  uint8_t const * point_elevation = &cloud->data.front() + elevationoff;
  for (V_PointCloudPoint::iterator iter = points_out.begin(); iter != points_out.end();
    ++iter, point_range += point_step,
    point_azimuth += point_step, point_elevation += point_step)
  {
    float rangeval = *reinterpret_cast<const float *>(point_range);
    float azimuthradian = DegreesToRadians(*reinterpret_cast<const float *>(point_azimuth));
    float elevationradian = DegreesToRadians(*reinterpret_cast<const float *>(point_elevation));

    iter->position.x = rangeval * cos(elevationradian) * cos(azimuthradian);
    iter->position.y = rangeval * cos(elevationradian) * sin(azimuthradian);
    iter->position.z = rangeval * sin(elevationradian);
  }

  return true;
}

}  // end namespace rviz_default_plugins
