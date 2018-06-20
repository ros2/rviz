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

#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__POINTCLOUD__POINT_CLOUD2_DISPLAY_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__POINTCLOUD__POINT_CLOUD2_DISPLAY_HPP_

#include <memory>

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "rviz_common/ros_topic_display.hpp"
#include "rviz_common/properties/queue_size_property.hpp"

#include "rviz_default_plugins/displays/pointcloud/point_cloud_common.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

namespace rviz_common
{
namespace properties
{
class IntProperty;
}
}

namespace rviz_default_plugins
{
namespace displays
{

struct Offsets
{
  uint32_t x, y, z;
};

// TODO(greimela) This display originally extended the MessageFilterDisplay. Revisit when available
/**
 * \class PointCloud2Display
 * \brief Displays a point cloud of type sensor_msgs::PointCloud2
 *
 * By default it will assume channel 0 of the cloud is an intensity value, and will color them by intensity.
 * If you set the channel's name to "rgb", it will interpret the channel as an integer rgb value, with r, g and b
 * all being 8 bits.
 */
class RVIZ_DEFAULT_PLUGINS_PUBLIC PointCloud2Display : public
  rviz_common::RosTopicDisplay<sensor_msgs::msg::PointCloud2>
{
public:
  PointCloud2Display();

  void reset() override;

  void update(float wall_dt, float ros_dt) override;

  /**
   * Filter any NAN values out of the cloud.  Any NAN values that make it through to PointCloudBase
   * will get their points put off in lala land, but it means they still do get processed/rendered
   * which can be a big performance hit
   * @param cloud The cloud to be filtered
   * @return A new cloud containing only the filtered points
   */
  sensor_msgs::msg::PointCloud2::ConstSharedPtr filterOutInvalidPoints(
    sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) const;

  /// Move to public for testing
  bool hasXYZChannels(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) const;

  /// Move to public for testing
  bool cloudDataMatchesDimensions(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) const;

  void onDisable() override;

protected:
  /** @brief Do initialization. Overridden from RosTopicDisplay. */
  void onInitialize() override;

  /** @brief Process a single message.  Overridden from RosTopicDisplay. */
  void processMessage(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) override;

private:
  std::unique_ptr<rviz_common::QueueSizeProperty> queue_size_property_;

  std::unique_ptr<PointCloudCommon> point_cloud_common_;

  sensor_msgs::msg::PointCloud2::_data_type
  filterData(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) const;

  bool validateFloatsAtPosition(
    sensor_msgs::msg::PointCloud2::_data_type::const_iterator position, Offsets offsets) const;

  Offsets determineOffsets(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) const;
};

}  // namespace displays
}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__POINTCLOUD__POINT_CLOUD2_DISPLAY_HPP_
