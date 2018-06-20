/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

#include "rviz_default_plugins/displays/pointcloud/point_cloud2_display.hpp"

#include <memory>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include "rviz_default_plugins/displays/pointcloud/point_cloud_common.hpp"
#include "rviz_default_plugins/displays/pointcloud/point_cloud_helpers.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/frame_manager_iface.hpp"
#include "rviz_rendering/objects/point_cloud.hpp"
#include "rviz_common/properties/int_property.hpp"
#include "rviz_common/validate_floats.hpp"
#include "rviz_common/properties/queue_size_property.hpp"
#include "rviz_common/uniform_string_stream.hpp"

namespace rviz_default_plugins
{
namespace displays
{

PointCloud2Display::PointCloud2Display()
: queue_size_property_(new rviz_common::QueueSizeProperty(this, 10)),
  point_cloud_common_(new PointCloudCommon(this))
{}

void PointCloud2Display::onInitialize()
{
  RTDClass::onInitialize();
  point_cloud_common_->initialize(context_, scene_node_);
}

void PointCloud2Display::processMessage(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
{
  if (!hasXYZChannels(cloud)) {
    return;
  }

  if (!cloudDataMatchesDimensions(cloud)) {
    rviz_common::UniformStringStream ss;
    ss << "Data size (" << cloud->data.size() << " bytes) does not match width (" << cloud->width <<
      ") times height (" << cloud->height << ") times point_step (" << cloud->point_step <<
      ").  Dropping message.";
    setStatusStd(rviz_common::properties::StatusProperty::Error, "Message", ss.str());
    return;
  }

  point_cloud_common_->addMessage(filterOutInvalidPoints(cloud));
}

bool PointCloud2Display::hasXYZChannels(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) const
{
  int32_t xi = findChannelIndex(cloud, "x");
  int32_t yi = findChannelIndex(cloud, "y");
  int32_t zi = findChannelIndex(cloud, "z");

  return xi != -1 && yi != -1 && zi != -1;
}

bool PointCloud2Display::cloudDataMatchesDimensions(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) const
{
  return cloud->width * cloud->height * cloud->point_step == cloud->data.size();
}

sensor_msgs::msg::PointCloud2::ConstSharedPtr PointCloud2Display::filterOutInvalidPoints(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) const
{
  auto filtered = std::make_shared<sensor_msgs::msg::PointCloud2>();

  if (cloud->width * cloud->height > 0) {
    filtered->data = filterData(cloud);
  }

  filtered->header = cloud->header;
  filtered->fields = cloud->fields;
  filtered->height = 1;
  filtered->width = static_cast<uint32_t>(filtered->data.size() / cloud->point_step);
  filtered->is_bigendian = cloud->is_bigendian;
  filtered->point_step = cloud->point_step;
  filtered->row_step = filtered->width;

  return filtered;
}

sensor_msgs::msg::PointCloud2::_data_type
PointCloud2Display::filterData(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) const
{
  sensor_msgs::msg::PointCloud2::_data_type filteredData;
  filteredData.reserve(cloud->data.size());

  Offsets offsets = determineOffsets(cloud);
  size_t points_to_copy = 0;
  sensor_msgs::msg::PointCloud2::_data_type::const_iterator copy_start_pos;
  for (auto it = cloud->data.begin(); it < cloud->data.end(); it += cloud->point_step) {
    if (validateFloatsAtPosition(it, offsets)) {
      if (points_to_copy == 0) {
        copy_start_pos = it;
      }
      ++points_to_copy;
    } else if (points_to_copy > 0) {
      filteredData.insert(
        filteredData.end(),
        copy_start_pos,
        copy_start_pos + points_to_copy * cloud->point_step);
      points_to_copy = 0;
    }
  }
  // Don't forget to flush what needs to be copied
  if (points_to_copy > 0) {
    filteredData.insert(
      filteredData.end(),
      copy_start_pos,
      copy_start_pos + points_to_copy * cloud->point_step);
  }

  return filteredData;
}

Offsets PointCloud2Display::determineOffsets(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) const
{
  Offsets offsets{
    cloud->fields[findChannelIndex(cloud, "x")].offset,
    cloud->fields[findChannelIndex(cloud, "y")].offset,
    cloud->fields[findChannelIndex(cloud, "z")].offset
  };
  return offsets;
}

bool PointCloud2Display::validateFloatsAtPosition(
  sensor_msgs::msg::PointCloud2::_data_type::const_iterator position,
  const Offsets offsets) const
{
  float x = *reinterpret_cast<const float *>(&*(position + offsets.x));
  float y = *reinterpret_cast<const float *>(&*(position + offsets.y));
  float z = *reinterpret_cast<const float *>(&*(position + offsets.z));

  return rviz_common::validateFloats(x) &&
         rviz_common::validateFloats(y) &&
         rviz_common::validateFloats(z);
}

void PointCloud2Display::update(float wall_dt, float ros_dt)
{
  point_cloud_common_->update(wall_dt, ros_dt);
}

void PointCloud2Display::reset()
{
  RTDClass::reset();
  point_cloud_common_->reset();
}

void PointCloud2Display::onDisable()
{
  RosTopicDisplay::onDisable();
  point_cloud_common_->onDisable();
}

}  // namespace displays
}  // namespace rviz_default_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::PointCloud2Display, rviz_common::Display)
