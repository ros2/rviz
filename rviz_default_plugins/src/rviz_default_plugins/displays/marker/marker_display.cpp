/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

#include "rviz_default_plugins/displays/marker/marker_display.hpp"

#include <memory>

namespace rviz_default_plugins
{
namespace displays
{

MarkerDisplay::MarkerDisplay()
: rviz_common::MessageFilterDisplay<visualization_msgs::msg::Marker>(),
  marker_common_(std::make_unique<MarkerCommon>(this))
{}

void MarkerDisplay::onInitialize()
{
  MFDClass::onInitialize();
  marker_common_->initialize(context_, scene_node_);
  topic_property_->setDescription(
    "visualization_msgs::msg::Marker topic to subscribe to. <topic>_array will also"
    " automatically be subscribed with type visualization_msgs::msg::MarkerArray.");
}

void MarkerDisplay::load(const rviz_common::Config & config)
{
  rviz_common::Display::load(config);
  marker_common_->load(config);
}

void MarkerDisplay::subscribe()
{
  MFDClass::subscribe();

  if ((!isEnabled()) || (topic_property_->getTopicStd().empty())) {
    return;
  }

  createMarkerArraySubscription();
}

void MarkerDisplay::createMarkerArraySubscription()
{
  try {
    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.event_callbacks.message_lost_callback =
      [&](rclcpp::QOSMessageLostInfo & info)
      {
        std::ostringstream sstm;
        sstm << "Some messages were lost:\n>\tNumber of new lost messages: " <<
          info.total_count_change << " \n>\tTotal number of messages lost: " <<
          info.total_count;
        setStatus(StatusLevel::Warn, "Array Topic", QString(sstm.str().c_str()));
      };

    // TODO(anhosi,wjwwood): replace with abstraction for subscriptions one available
    array_sub_ = rviz_ros_node_.lock()->get_raw_node()->
      template create_subscription<visualization_msgs::msg::MarkerArray>(
      topic_property_->getTopicStd() + "_array",
      qos_profile,
      [this](visualization_msgs::msg::MarkerArray::ConstSharedPtr msg) {
        marker_common_->addMessage(msg);
      },
      sub_opts);
    setStatus(StatusLevel::Ok, "Array Topic", "OK");
  } catch (rclcpp::exceptions::InvalidTopicNameError & e) {
    setStatus(StatusLevel::Error, "Array Topic", QString("Error subscribing: ") + e.what());
  }
}

void MarkerDisplay::unsubscribe()
{
  MFDClass::unsubscribe();
  array_sub_.reset();
}


void MarkerDisplay::processMessage(const visualization_msgs::msg::Marker::ConstSharedPtr msg)
{
  marker_common_->addMessage(msg);
}

void MarkerDisplay::update(float wall_dt, float ros_dt)
{
  marker_common_->update(wall_dt, ros_dt);
}

void MarkerDisplay::reset()
{
  MFDClass::reset();
  marker_common_->clearMarkers();
}

}  // namespace displays
}  // namespace rviz_default_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::MarkerDisplay, rviz_common::Display)
