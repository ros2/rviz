/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

#include "rviz_default_plugins/displays/pointcloud/point_cloud_to_point_cloud2.hpp"

#include <memory>

uint32_t size(size_t value)
{
  return static_cast<uint32_t>(value);
}

sensor_msgs::msg::PointCloud2::ConstSharedPtr rviz_default_plugins::convertPointCloudToPointCloud2(
  const sensor_msgs::msg::PointCloud::ConstSharedPtr input)
{
  sensor_msgs::msg::PointCloud2::SharedPtr output(
    new sensor_msgs::msg::PointCloud2_<std::allocator<void>>());
  output->header = input->header;
  output->width = size(input->points.size());
  output->height = 1;
  output->fields.resize(3 + input->channels.size());

  // Convert x/y/z to fields
  output->fields[0].name = "x"; output->fields[1].name = "y"; output->fields[2].name = "z";

  size_t offset = 0;
  for (size_t d = 0; d < output->fields.size(); ++d, offset += sizeof(float)) {
    output->fields[d].offset = size(offset);
    output->fields[d].datatype = sensor_msgs::msg::PointField::FLOAT32;
  }
  output->point_step = size(offset);
  output->row_step = output->point_step * output->width;
  // Convert the remaining of the channels to fields
  for (size_t d = 0; d < input->channels.size(); ++d) {
    output->fields[3 + d].name = input->channels[d].name;
  }
  output->data.resize(input->points.size() * output->point_step);
  output->is_bigendian = false;
  output->is_dense = false;

  // Copy the data points
  auto floatData = reinterpret_cast<float *>(output->data.data());
  for (size_t cp = 0; cp < input->points.size(); ++cp) {
    floatData[(cp * output->point_step + output->fields[0].offset) / sizeof(float)] =
      input->points[cp].x;
    floatData[(cp * output->point_step + output->fields[1].offset) / sizeof(float)] =
      input->points[cp].y;
    floatData[(cp * output->point_step + output->fields[2].offset) / sizeof(float)] =
      input->points[cp].z;

    for (size_t d = 0; d < input->channels.size(); ++d) {
      if (input->channels[d].values.size() == input->points.size()) {
        floatData[(cp * output->point_step + output->fields[3 + d].offset) / sizeof(float)] =
          input->channels[d].values[cp];
      }
    }
  }
  return output;
}
