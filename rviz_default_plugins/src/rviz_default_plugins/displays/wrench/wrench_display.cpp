/*
 * Copyright (c) 2019, Martin Idel and others
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

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include "rviz_common/display_context.hpp"
#include "rviz_common/frame_manager_iface.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/int_property.hpp"
#include "rviz_common/properties/parse_color.hpp"
#include "rviz_common/validate_floats.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_rendering/objects/wrench_visual.hpp"

#include "rviz_default_plugins/displays/wrench/wrench_display.hpp"

namespace rviz_default_plugins
{
namespace displays
{

WrenchDisplay::WrenchDisplay()
{
  accept_nan_values_ = new rviz_common::properties::BoolProperty(
    "Accept NaN Values", false,
    "NaN values in incoming messages are converted to 0 to display wrench vector.", this,
    SLOT(updateWrenchVisuals()));

  force_color_property_ = new rviz_common::properties::ColorProperty(
    "Force Color", QColor(204, 51, 51), "Color to draw the force arrows.", this,
    SLOT(updateWrenchVisuals()));

  torque_color_property_ = new rviz_common::properties::ColorProperty(
    "Torque Color", QColor(204, 204, 51), "Color to draw the torque arrows.", this,
    SLOT(updateWrenchVisuals()));

  alpha_property_ = new rviz_common::properties::FloatProperty(
    "Alpha", 1.0f, "0 is fully transparent, 1.0 is fully opaque.", this,
    SLOT(updateWrenchVisuals()));

  force_scale_property_ = new rviz_common::properties::FloatProperty(
    "Force Arrow Scale", 2.0f, "force arrow scale", this, SLOT(updateWrenchVisuals()));

  torque_scale_property_ = new rviz_common::properties::FloatProperty(
    "Torque Arrow Scale", 2.0f, "torque arrow scale", this, SLOT(updateWrenchVisuals()));

  width_property_ = new rviz_common::properties::FloatProperty(
    "Arrow Width", 0.5f, "arrow width", this, SLOT(updateWrenchVisuals()));

  history_length_property_ = new rviz_common::properties::IntProperty(
    "History Length", 1, "Number of prior measurements to display.", this,
    SLOT(updateHistoryLength()));

  history_length_property_->setMin(1);
  history_length_property_->setMax(100000);
}

void WrenchDisplay::onInitialize()
{
  MFDClass::onInitialize();
  updateHistoryLength();
}

WrenchDisplay::~WrenchDisplay() = default;

void WrenchDisplay::reset()
{
  MFDClass::reset();
  visuals_.clear();
}

void WrenchDisplay::updateWrenchVisuals()
{
  float alpha = alpha_property_->getFloat();
  float force_scale = force_scale_property_->getFloat();
  float torque_scale = torque_scale_property_->getFloat();
  float width = width_property_->getFloat();
  Ogre::ColourValue force_color = force_color_property_->getOgreColor();
  Ogre::ColourValue torque_color = torque_color_property_->getOgreColor();

  for (const auto & visual : visuals_) {
    visual->setForceColor(force_color.r, force_color.g, force_color.b, alpha);
    visual->setTorqueColor(torque_color.r, torque_color.g, torque_color.b, alpha);
    visual->setForceScale(force_scale);
    visual->setTorqueScale(torque_scale);
    visual->setWidth(width);
  }
}

void WrenchDisplay::updateHistoryLength()
{
  while (visuals_.size() > static_cast<size_t>(history_length_property_->getInt())) {
    visuals_.pop_front();
  }
}

bool validateFloats(const geometry_msgs::msg::WrenchStamped & msg)
{
  return rviz_common::validateFloats(msg.wrench.force) &&
         rviz_common::validateFloats(msg.wrench.torque);
}

void WrenchDisplay::processMessage(geometry_msgs::msg::WrenchStamped::ConstSharedPtr msg)
{
  auto adjusted_msg = std::make_shared<geometry_msgs::msg::WrenchStamped>();
  bool accept_nan = accept_nan_values_->getBool();

  if (!accept_nan) {
    if (!validateFloats(*msg)) {
      setStatus(
        rviz_common::properties::StatusProperty::Error, "Topic",
        "Message contained invalid floating point values (nans or infs)");
      return;
    }
  } else {
    adjusted_msg->wrench.force.x = (std::isnan(msg->wrench.force.x)) ? 0.0 : msg->wrench.force.x;
    adjusted_msg->wrench.force.y = (std::isnan(msg->wrench.force.y)) ? 0.0 : msg->wrench.force.y;
    adjusted_msg->wrench.force.z = (std::isnan(msg->wrench.force.z)) ? 0.0 : msg->wrench.force.z;

    adjusted_msg->wrench.torque.x = (std::isnan(msg->wrench.torque.x)) ? 0.0 :
      msg->wrench.torque.x;
    adjusted_msg->wrench.torque.y = (std::isnan(msg->wrench.torque.y)) ? 0.0 :
      msg->wrench.torque.y;
    adjusted_msg->wrench.torque.z = (std::isnan(msg->wrench.torque.z)) ? 0.0 :
      msg->wrench.torque.z;

    if (!validateFloats(*msg)) {
      setStatus(
        rviz_common::properties::StatusProperty::Error, "Topic",
        "Message contained invalid floating point values (nans or infs)");
      return;
    }
  }

  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  rclcpp::Time time_stamp(msg->header.stamp, RCL_ROS_TIME);
  if (!context_->getFrameManager()->getTransform(
      msg->header.frame_id, time_stamp, position, orientation))
  {
    setMissingTransformToFixedFrame(msg->header.frame_id);
    return;
  }

  if (position.isNaN()) {
    RVIZ_COMMON_LOG_ERROR(
      "Wrench position contains NaNs. Skipping render as long as the position is invalid");
    return;
  }

  if (visuals_.size() >= static_cast<size_t>(history_length_property_->getInt())) {
    visuals_.pop_front();
  }

  auto visual = (!accept_nan) ? createWrenchVisual(msg, orientation, position) :
    createWrenchVisual(adjusted_msg, orientation, position);

  visuals_.push_back(visual);
}

std::shared_ptr<rviz_rendering::WrenchVisual> WrenchDisplay::createWrenchVisual(
  const geometry_msgs::msg::WrenchStamped::ConstSharedPtr & msg,
  const Ogre::Quaternion & orientation,
  const Ogre::Vector3 & position)
{
  std::shared_ptr<rviz_rendering::WrenchVisual> visual;
  visual = std::make_shared<rviz_rendering::WrenchVisual>(context_->getSceneManager(), scene_node_);

  Ogre::Vector3 force(
    static_cast<float>(msg->wrench.force.x),
    static_cast<float>(msg->wrench.force.y),
    static_cast<float>(msg->wrench.force.z));
  Ogre::Vector3 torque(
    static_cast<float>(msg->wrench.torque.x),
    static_cast<float>(msg->wrench.torque.y),
    static_cast<float>(msg->wrench.torque.z));
  visual->setWrench(force, torque);
  visual->setFramePosition(position);
  visual->setFrameOrientation(orientation);

  float alpha = alpha_property_->getFloat();
  float force_scale = force_scale_property_->getFloat();
  float torque_scale = torque_scale_property_->getFloat();
  float width = width_property_->getFloat();
  Ogre::ColourValue force_color = force_color_property_->getOgreColor();
  Ogre::ColourValue torque_color = torque_color_property_->getOgreColor();

  visual->setForceColor(force_color.r, force_color.g, force_color.b, alpha);
  visual->setTorqueColor(torque_color.r, torque_color.g, torque_color.b, alpha);
  visual->setForceScale(force_scale);
  visual->setTorqueScale(torque_scale);
  visual->setWidth(width);

  return visual;
}

}  // namespace displays
}  // namespace rviz_default_plugins

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::WrenchDisplay, rviz_common::Display)
