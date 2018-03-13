/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

#include "laser_scan_display.hpp"

#include <memory>
#include <string>

#include "laser_geometry/laser_geometry.hpp"
#include "tf2_ros/buffer.h"

#include "src/rviz_default_plugins/displays/pointcloud/point_cloud_common.hpp"
#include "rviz_common/properties/int_property.hpp"
#include "rviz_common/properties/queue_size_property.hpp"
#include "rviz_common/validate_floats.hpp"

namespace rviz_default_plugins
{
namespace displays
{

LaserScanDisplay::LaserScanDisplay()
: point_cloud_common_(std::make_unique<rviz_default_plugins::PointCloudCommon>(this)),
  queue_size_property_(std::make_unique<rviz_common::QueueSizeProperty>(this, 10)),
  projector_(std::make_unique<laser_geometry::LaserProjection>())
{}

void LaserScanDisplay::onInitialize()
{
  RTDClass::onInitialize();
  point_cloud_common_->initialize(context_, scene_node_);
}

void LaserScanDisplay::processMessage(sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
{
  auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();

  try {
    auto buffer = context_->getFrameManager()->getTFBufferPtr();
    projector_->transformLaserScanToPointCloud(
      fixed_frame_.toStdString(),
      *scan,
      *cloud,
      *buffer,
      laser_geometry::channel_option::Intensity);
  } catch (tf2::TransformException & e) {
    RVIZ_COMMON_LOG_DEBUG_STREAM(
      "LaserScan [" << qPrintable(getName()) << "]: failed to transform scan:" << e.what() << ".");
    return;
  }

  point_cloud_common_->addMessage(cloud);
}

void LaserScanDisplay::update(float wall_dt, float ros_dt)
{
  point_cloud_common_->update(wall_dt, ros_dt);
}

void LaserScanDisplay::reset()
{
  RTDClass::reset();
  point_cloud_common_->reset();
}

}  // namespace displays
}  // namespace rviz_default_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::LaserScanDisplay, rviz_common::Display)
