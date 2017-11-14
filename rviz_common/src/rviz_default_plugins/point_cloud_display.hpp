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

#ifndef RVIZ_DEFAULT_PLUGINS__POINT_CLOUD_DISPLAY_HPP_
#define RVIZ_DEFAULT_PLUGINS__POINT_CLOUD_DISPLAY_HPP_

#include <deque>
#include <queue>
#include <vector>

#include "sensor_msgs/msg/point_cloud.hpp"

#include "rviz_common/message_filter_display.hpp"

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

/**
 * \class PointCloudDisplay
 * \brief Displays a point cloud of type sensor_msgs::PointCloud
 *
 * By default it will assume channel 0 of the cloud is an intensity value, and will color them by intensity.
 * If you set the channel's name to "rgb", it will interpret the channel as an integer rgb value, with r, g and b
 * all being 8 bits.
 */
class PointCloudDisplay : public rviz_common::MessageFilterDisplay<sensor_msgs::msg::PointCloud>
{
  Q_OBJECT

public:
  PointCloudDisplay();
  ~PointCloudDisplay();

  virtual void reset();

  virtual void update(float wall_dt, float ros_dt);

private Q_SLOTS:
  void updateQueueSize();

protected:
  /** @brief Do initialization. Overridden from MessageFilterDisplay. */
  virtual void onInitialize();

  /** @brief Process a single message.  Overridden from MessageFilterDisplay. */
  virtual void processMessage(const sensor_msgs::msg::PointCloud::ConstSharedPtr & cloud);

  rviz_common::properties::IntProperty * queue_size_property_;

  PointCloudCommon * point_cloud_common_;
};

}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__POINT_CLOUD_DISPLAY_HPP_
