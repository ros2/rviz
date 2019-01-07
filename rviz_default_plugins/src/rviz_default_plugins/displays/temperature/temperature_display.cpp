/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

#include "rviz_default_plugins/displays/temperature/temperature_display.hpp"

namespace rviz_default_plugins
{

namespace displays
{

TemperatureDisplay::TemperatureDisplay()
{}

TemperatureDisplay::~TemperatureDisplay() = default;

void TemperatureDisplay::setInitialValues()
{
  subProp("Channel Name")->setValue("temperature");
  subProp("Autocompute Intensity Bounds")->setValue(false);
  subProp("Invert Rainbow")->setValue(true);
  subProp("Min Intensity")->setValue(0);
  subProp("Max Intensity")->setValue(100);
}

void TemperatureDisplay::hideUnneededProperties()
{
  subProp("Position Transformer")->hide();
  subProp("Color Transformer")->hide();
  subProp("Channel Name")->hide();
  subProp("Invert Rainbow")->hide();
  subProp("Autocompute Intensity Bounds")->hide();
}

void TemperatureDisplay::processMessage(sensor_msgs::msg::Temperature::ConstSharedPtr message)
{
  auto point_cloud2_message =
    createPointCloud2Message(message->header, message->temperature, "temperature");

  point_cloud_common_->addMessage(point_cloud2_message);
}

}  // namespace displays
}  // namespace rviz_default_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::TemperatureDisplay, rviz_common::Display)
