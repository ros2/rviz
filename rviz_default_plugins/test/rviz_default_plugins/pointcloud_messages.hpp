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

#ifndef RVIZ_DEFAULT_PLUGINS__POINTCLOUD_MESSAGES_HPP_
#define RVIZ_DEFAULT_PLUGINS__POINTCLOUD_MESSAGES_HPP_

#include <string>
#include <vector>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"

namespace rviz_default_plugins
{

struct Point
{
  float x, y, z;
  Point(float x, float y, float z)
  : x(x), y(y), z(z) {}
};

struct PointWithIntensity : Point
{
  float intensity;
  PointWithIntensity(float x, float y, float z, float intensity)
  : Point(x, y, z), intensity(intensity) {}
};

struct ColoredPoint : Point
{
  float r, g, b;
  ColoredPoint(float x, float y, float z, float r, float g, float b)
  : Point(x, y, z), r(r), g(g), b(b) {}
};

sensor_msgs::msg::PointCloud::ConstSharedPtr createPointCloudWithSquare();
sensor_msgs::msg::PointCloud::ConstSharedPtr createPointCloudWithPoints(std::vector<Point> points);

sensor_msgs::msg::PointCloud2::ConstSharedPtr createPointCloud2WithSquare();
sensor_msgs::msg::PointCloud2::SharedPtr createPointCloud2WithPoints(
  const std::vector<Point> & points);
sensor_msgs::msg::PointCloud2::ConstSharedPtr createF32ColoredPointCloud2(
  const std::vector<ColoredPoint> & points);
sensor_msgs::msg::PointCloud2::ConstSharedPtr create8BitColoredPointCloud2(
  const std::vector<ColoredPoint> & points);
sensor_msgs::msg::PointCloud2::ConstSharedPtr createPointCloud2WithIntensity(
  const std::vector<PointWithIntensity> & points);

}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__POINTCLOUD_MESSAGES_HPP_
