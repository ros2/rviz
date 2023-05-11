/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * Copyright (c) 2018, TNG Technology Consulting GmbH.
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

#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__RELATIVE_HUMIDITY__RELATIVE_HUMIDITY_DISPLAY_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__RELATIVE_HUMIDITY__RELATIVE_HUMIDITY_DISPLAY_HPP_

#include "rviz_default_plugins/displays/pointcloud/point_cloud_scalar_display.hpp"
#include "sensor_msgs/msg/relative_humidity.hpp"

namespace rviz_default_plugins
{

class PointCloudCommon;

namespace displays
{

/// Display a RelativeHumidity message of type sensor_msgs::RelativeHumidity
/**
 * \class RelativeHumidityDisplay
 */

class RVIZ_DEFAULT_PLUGINS_PUBLIC RelativeHumidityDisplay
  : public PointCloudScalarDisplay<sensor_msgs::msg::RelativeHumidity>
{
  Q_OBJECT

public:
  RelativeHumidityDisplay();
  ~RelativeHumidityDisplay() override;

protected:
  void processMessage(const sensor_msgs::msg::RelativeHumidity::ConstSharedPtr message) override;

private:
  void setInitialValues() override;
  void hideUnneededProperties() override;
};

}  // namespace displays
}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__RELATIVE_HUMIDITY__RELATIVE_HUMIDITY_DISPLAY_HPP_
