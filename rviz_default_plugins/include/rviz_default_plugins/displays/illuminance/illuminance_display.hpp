/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * Copyright (c) 2018, Maximilian Kuehn
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

#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__ILLUMINANCE__ILLUMINANCE_DISPLAY_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__ILLUMINANCE__ILLUMINANCE_DISPLAY_HPP_

#include <memory>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/illuminance.hpp"

#include "rviz_common/ros_topic_display.hpp"
#include "rviz_common/properties/queue_size_property.hpp"
#include "rviz_default_plugins/displays/pointcloud/point_cloud_common.hpp"


#include "rviz_default_plugins/visibility_control.hpp"


namespace rviz_common
{
namespace properties
{
  class IntProperty;
}  // namespace properties
}  // namespace rviz_common

namespace rviz_default_plugins
{

class PointCloudCommon;

namespace displays
{

/**
 * \class IlluminanceDisplay
 * \brief Displays a Illuminance message of type sensor_msgs::Illuminance
 *
 */

class RVIZ_DEFAULT_PLUGINS_PUBLIC IlluminanceDisplay
: public rviz_common::RosTopicDisplay<sensor_msgs::msg::Illuminance>
{
  Q_OBJECT

public:
  IlluminanceDisplay();
  ~IlluminanceDisplay();

  void reset() override;
  void update(float wall_dt, float ros_dt) override;
  void onDisable() override;
  void processMessage(const sensor_msgs::msg::Illuminance::ConstSharedPtr message) override;

protected:
  void onInitialize() override;

private:
  std::unique_ptr<rviz_common::QueueSizeProperty> queue_size_property_;
  std::shared_ptr<PointCloudCommon> point_cloud_common_;
};

}  // namespace displays
}  // namespace rviz_default_plugins

#endif // RVIZ_DEFAULT_PLUGINS__DISPLAYS__ILLUMINANCE__ILLUMINANCE_DISPLAY_HPP_
