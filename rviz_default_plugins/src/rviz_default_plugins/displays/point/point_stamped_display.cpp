/*
 * Copyright (c) 2014, others
 * Copyright (c) 2018, Bosch Software Innovations GmbH.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "rviz_default_plugins/displays/point/point_stamped_display.hpp"

#include <deque>
#include <memory>
#include <string>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include "rviz_rendering/objects/shape.hpp"

#include "rviz_common/display_context.hpp"
#include "rviz_common/msg_conversions.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/int_property.hpp"
#include "rviz_common/frame_manager_iface.hpp"
#include "rviz_common/validate_floats.hpp"
#include "rviz_common/logging.hpp"

namespace rviz_default_plugins
{
namespace displays
{

PointStampedDisplay::PointStampedDisplay(rviz_common::DisplayContext * context)
{
  context_ = context;
  scene_manager_ = context_->getSceneManager();
  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  setUpProperties();
}

PointStampedDisplay::PointStampedDisplay()
{
  setUpProperties();
}

void PointStampedDisplay::onInitialize()
{
  MFDClass::onInitialize();
}

void PointStampedDisplay::setUpProperties()
{
  color_property_ = new rviz_common::properties::ColorProperty(
    "Color", QColor(204, 41, 204), "Color of a point", this, SLOT(updateColorAndAlpha()));

  alpha_property_ = new rviz_common::properties::FloatProperty(
    "Alpha", 1.0f, "0 is fully transparent, 1.0 is fully opaque.",
    this, SLOT(updateColorAndAlpha()));

  radius_property_ = new rviz_common::properties::FloatProperty(
    "Radius", 0.2f, "Radius of a point", this, SLOT(updateColorAndAlpha()));

  history_length_property_ = new rviz_common::properties::IntProperty(
    "History Length", 1, "Number of prior measurements to display.",
    this, SLOT(onlyKeepHistoryLengthNumberOfVisuals()));
  history_length_property_->setMin(1);
  history_length_property_->setMax(100000);
}

PointStampedDisplay::~PointStampedDisplay() = default;

void PointStampedDisplay::reset()
{
  MFDClass::reset();
  visuals_.clear();
}

void PointStampedDisplay::updateColorAndAlpha()
{
  float alpha = alpha_property_->getFloat();
  float radius = radius_property_->getFloat();
  Ogre::ColourValue color = color_property_->getOgreColor();

  for (auto visual : visuals_) {
    visual->setColor(color.r, color.g, color.b, alpha);
    visual->setScale(Ogre::Vector3(radius, radius, radius));
  }
}

void PointStampedDisplay::onlyKeepHistoryLengthNumberOfVisuals()
{
  while (visuals_.size() > static_cast<size_t>(history_length_property_->getInt())) {
    visuals_.pop_front();
  }
}

void PointStampedDisplay::processMessage(geometry_msgs::msg::PointStamped::ConstSharedPtr msg)
{
  if (!rviz_common::validateFloats(msg->point)) {
    setStatus(
      rviz_common::properties::StatusProperty::Error, "Topic",
      "Message contained invalid floating point values (nans or infs)");
    return;
  }

  rclcpp::Time time_stamp(msg->header.stamp, RCL_ROS_TIME);
  if (!updateFrame(msg->header.frame_id, time_stamp)) {
    setMissingTransformToFixedFrame(msg->header.frame_id);
    return;
  }
  setTransformOk();

  if (visuals_.size() >= static_cast<size_t>(history_length_property_->getInt())) {
    visuals_.pop_front();
  }

  createNewSphereVisual(msg);
}

void PointStampedDisplay::createNewSphereVisual(
  const geometry_msgs::msg::PointStamped::ConstSharedPtr & msg)
{
  std::shared_ptr<rviz_rendering::Shape> visual = std::make_shared<rviz_rendering::Shape>(
    rviz_rendering::Shape::Sphere, context_->getSceneManager(), scene_node_);

  float alpha = alpha_property_->getFloat();
  float radius = radius_property_->getFloat();
  Ogre::ColourValue color = color_property_->getOgreColor();
  visual->setColor(color.r, color.g, color.b, alpha);
  visual->setPosition(rviz_common::pointMsgToOgre(msg->point));
  visual->setScale(Ogre::Vector3(radius, radius, radius));

  visuals_.push_back(visual);
}

}  // namespace displays
}  // namespace rviz_default_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::PointStampedDisplay, rviz_common::Display)
