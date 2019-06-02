/*
 * Copyright (c) 2011, Willow Garage, Inc.'
 * Copyright (c) 2018, Bosch Software Innovations GmbH.
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

#include <memory>

#include "rviz_default_plugins/displays/marker_array/marker_array_display.hpp"

#include "rviz_common/properties/int_property.hpp"
#include "rviz_common/properties/ros_topic_property.hpp"

namespace rviz_default_plugins
{
namespace displays
{

MarkerArrayDisplay::MarkerArrayDisplay()
: rviz_common::RosTopicDisplay<visualization_msgs::msg::MarkerArray>(),
  marker_common_(std::make_unique<MarkerCommon>(this))
{}

void MarkerArrayDisplay::onInitialize()
{
  RTDClass::onInitialize();
  marker_common_->initialize(context_, scene_node_);

  topic_property_->setValue("visualization_marker_array");
  topic_property_->setDescription("visualization_msgs::MarkerArray topic to subscribe to.");
}

void MarkerArrayDisplay::load(const rviz_common::Config & config)
{
  Display::load(config);
  marker_common_->load(config);
}

void MarkerArrayDisplay::processMessage(visualization_msgs::msg::MarkerArray::ConstSharedPtr msg)
{
  marker_common_->addMessage(msg);
}

void MarkerArrayDisplay::update(float wall_dt, float ros_dt)
{
  marker_common_->update(wall_dt, ros_dt);
}

void MarkerArrayDisplay::reset()
{
  RosTopicDisplay::reset();
  marker_common_->clearMarkers();
}

}  // end namespace displays
}  // end namespace rviz_default_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::MarkerArrayDisplay, rviz_common::Display)
