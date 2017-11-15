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

#include "message_creators.hpp"

#include <memory>
#include <vector>

#include "rclcpp/time.hpp"

namespace rviz_default_plugins
{

sensor_msgs::msg::PointCloud2::ConstSharedPtr createPointCloud2WithPoints(std::vector<Point> points)
{
  auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
  cloud->header = std_msgs::msg::Header();
  cloud->header.stamp = rclcpp::Time::now();

  cloud->is_bigendian = false;
  cloud->is_dense = true;

  cloud->height = 1;
  cloud->width = (uint32_t) points.size();

  cloud->fields.resize(3);
  cloud->fields[0].name = "x";
  cloud->fields[1].name = "y";
  cloud->fields[2].name = "z";

  sensor_msgs::msg::PointField::_offset_type offset = 0;
  for (uint32_t i = 0; i < cloud->fields.size(); ++i, offset += sizeof(float)) {
    cloud->fields[i].count = 1;
    cloud->fields[i].offset = offset;
    cloud->fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
  }

  cloud->point_step = offset;
  cloud->row_step = cloud->point_step * cloud->width;
  cloud->data.resize(cloud->row_step * cloud->height);

  for (uint32_t i = 0; i < cloud->width; ++i) {
    memcpy(
      &cloud->data[i * cloud->point_step + cloud->fields[0].offset], &points[i].x, sizeof(float));
    memcpy(
      &cloud->data[i * cloud->point_step + cloud->fields[1].offset], &points[i].y, sizeof(float));
    memcpy(
      &cloud->data[i * cloud->point_step + cloud->fields[2].offset], &points[i].z, sizeof(float));
  }

  return cloud;
}

sensor_msgs::msg::PointCloud::ConstSharedPtr createPointCloudWithPoints(std::vector<Point> points)
{
  auto message = sensor_msgs::msg::PointCloud();
  message.header = std_msgs::msg::Header();
  message.header.stamp = rclcpp::Time::now();
  message.header.frame_id = "base_link";

  std::vector<geometry_msgs::msg::Point32> points32;
  for (auto const & point : points) {
    geometry_msgs::msg::Point32 p;
    p.x = point.x;
    p.y = point.y;
    p.z = point.z;
    message.points.push_back(p);
  }

  return std::make_shared<sensor_msgs::msg::PointCloud>(message);
}

static std::vector<Point> points = {{1, 1, 1}, {-1, -1, 1}, {-1, 1, 1}, {1, -1, 1}};

sensor_msgs::msg::PointCloud2::ConstSharedPtr createPointCloud2WithSquare()
{
  return createPointCloud2WithPoints(points);
}

sensor_msgs::msg::PointCloud::ConstSharedPtr createPointCloudWithSquare()
{
  return createPointCloudWithPoints(points);
}

}  // namespace rviz_default_plugins
