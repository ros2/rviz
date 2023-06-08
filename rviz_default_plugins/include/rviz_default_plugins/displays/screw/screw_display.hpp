/*
 * Copyright (c) 2023, Open Source Robotics Foundation, Inc.
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

#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__SCREW__SCREW_DISPLAY_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__SCREW__SCREW_DISPLAY_HPP_

#include <deque>
#include <memory>

#include <geometry_msgs/msg/vector3.hpp>
#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_rendering/objects/screw_visual.hpp>
#include <std_msgs/msg/header.hpp>


#include "rviz_default_plugins/visibility_control.hpp"

namespace rviz_default_plugins
{
namespace displays
{
template<class MessageType>
class RVIZ_DEFAULT_PLUGINS_PUBLIC ScrewDisplay
  : public rviz_common::MessageFilterDisplay<MessageType>
{
public:
  // Constructor. pluginlib::ClassLoader creates instances by calling
  // the default constructor, so make sure you have one.
  ScrewDisplay();
  ~ScrewDisplay() override = default;

protected:
  // Overrides of public virtual functions from the Display class.
  void onInitialize() override;
  void reset() override;

  // Helper function to properties for all visuals.
  void updateProperties();
  void updateHistoryLength();

  void processMessagePrivate(
    const std_msgs::msg::Header & header,
    const geometry_msgs::msg::Vector3 & linear,
    const geometry_msgs::msg::Vector3 & angular);

  // Storage for the list of visuals par each joint intem
  // Storage for the list of visuals. It is a circular buffer where
  // data gets popped from the front (oldest) and pushed to the back (newest)
  std::deque<std::shared_ptr<rviz_rendering::ScrewVisual>> visuals_;

  // Property objects for user-editable properties.
  rviz_common::properties::ColorProperty * linear_color_property_;
  rviz_common::properties::ColorProperty * angular_color_property_;
  rviz_common::properties::FloatProperty * alpha_property_;
  rviz_common::properties::FloatProperty * linear_scale_property_;
  rviz_common::properties::FloatProperty * angular_scale_property_;
  rviz_common::properties::FloatProperty * width_property_;
  rviz_common::properties::IntProperty * history_length_property_;
  rviz_common::properties::BoolProperty * hide_small_values_property_;
};

}  // namespace displays
}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__SCREW__SCREW_DISPLAY_HPP_
