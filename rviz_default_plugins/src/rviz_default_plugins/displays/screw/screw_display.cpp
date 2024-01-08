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

#include "rviz_default_plugins/displays/screw/screw_display.hpp"

#include <QObject>
#include <QString>
#include <memory>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/accel_stamped.hpp>
#include <rviz_common/validate_floats.hpp>
#include <rviz_common/logging.hpp>

template<class MessageType>
constexpr const char * linear()
{
  return "Linear";
}
template<class MessageType>
constexpr const char * angular()
{
  return "Angular";
}

namespace rviz_default_plugins
{
namespace displays
{
template<class MessageType>
ScrewDisplay<MessageType>::ScrewDisplay()
{
  auto lin = linear<MessageType>();
  auto ang = angular<MessageType>();
  linear_color_property_ = new rviz_common::properties::ColorProperty(
    QString("%1 Color").arg(lin), QColor(204, 51, 51),
    QString("Color to draw the %1 arrows.").arg(QString(lin).toLower()),
    this);

  QObject::connect(
    linear_color_property_, &rviz_common::properties::Property::changed,
    this, &ScrewDisplay<MessageType>::updateProperties);

  angular_color_property_ = new rviz_common::properties::ColorProperty(
    QString("%1 Color").arg(ang), QColor(204, 204, 51),
    QString("Color to draw the %1 arrows.").arg(QString(ang).toLower()),
    this);
  QObject::connect(
    angular_color_property_, &rviz_common::properties::Property::changed,
    this, &ScrewDisplay<MessageType>::updateProperties);

  alpha_property_ = new rviz_common::properties::FloatProperty(
    "Alpha", 1.0, "0 is fully transparent, 1.0 is fully opaque.",
    this);
  QObject::connect(
    alpha_property_, &rviz_common::properties::Property::changed,
    this, &ScrewDisplay<MessageType>::updateProperties);

  linear_scale_property_ = new rviz_common::properties::FloatProperty(
    QString("%1 Arrow Scale").arg(lin), 2.0,
    QString("%1 arrow scale").arg(lin), this);

  QObject::connect(
    linear_scale_property_, &rviz_common::properties::Property::changed,
    this, &ScrewDisplay<MessageType>::updateProperties);

  angular_scale_property_ = new rviz_common::properties::FloatProperty(
    QString("%1 Arrow Scale").arg(ang), 2.0,
    QString("%1 arrow scale").arg(ang), this);

  QObject::connect(
    angular_scale_property_, &rviz_common::properties::Property::changed,
    this, &ScrewDisplay<MessageType>::updateProperties);

  width_property_ = new rviz_common::properties::FloatProperty(
    "Arrow Width", 0.5, "arrow width", this);

  QObject::connect(
    width_property_, &rviz_common::properties::Property::changed,
    this, &ScrewDisplay<MessageType>::updateProperties);

  history_length_property_ = new rviz_common::properties::IntProperty(
    "History Length", 1, "Number of prior measurements to display.", this);

  QObject::connect(
    history_length_property_, &rviz_common::properties::Property::changed,
    this, &ScrewDisplay<MessageType>::updateHistoryLength);

  hide_small_values_property_ = new rviz_common::properties::BoolProperty(
    "Hide Small Values", true, "Hide small values", this);

  QObject::connect(
    hide_small_values_property_, &rviz_common::properties::Property::changed,
    this, &ScrewDisplay<MessageType>::updateProperties);

  history_length_property_->setMin(1);
  history_length_property_->setMax(100000);
}

template<class MessageType>
void ScrewDisplay<MessageType>::onInitialize()
{
  rviz_common::MessageFilterDisplay<MessageType>::onInitialize();
  updateHistoryLength();
}

// Override rviz::Display's reset() function to add a call to clear().
template<class MessageType>
void ScrewDisplay<MessageType>::reset()
{
  rviz_common::MessageFilterDisplay<MessageType>::reset();
  visuals_.clear();
}

template<class MessageType>
void ScrewDisplay<MessageType>::updateProperties()
{
  float alpha = alpha_property_->getFloat();
  float linear_scale = linear_scale_property_->getFloat();
  float angular_scale = angular_scale_property_->getFloat();
  float width = width_property_->getFloat();
  bool hide_small_values = hide_small_values_property_->getBool();
  Ogre::ColourValue linear_color = linear_color_property_->getOgreColor();
  Ogre::ColourValue angular_color = angular_color_property_->getOgreColor();

  for (size_t i = 0; i < visuals_.size(); i++) {
    visuals_[i]->setLinearColor(linear_color.r, linear_color.g, linear_color.b, alpha);
    visuals_[i]->setAngularColor(angular_color.r, angular_color.g, angular_color.b, alpha);
    visuals_[i]->setLinearScale(linear_scale);
    visuals_[i]->setAngularScale(angular_scale);
    visuals_[i]->setWidth(width);
    visuals_[i]->setHideSmallValues(hide_small_values);
  }
}

// Set the number of past visuals to show.
template<class MessageType>
void ScrewDisplay<MessageType>::updateHistoryLength()
{
  while (visuals_.size() > static_cast<size_t>(history_length_property_->getInt())) {
    visuals_.pop_front();
  }
}

// This is our callback to handle an incoming message.
template<class MessageType>
void ScrewDisplay<MessageType>::processMessagePrivate(
  const std_msgs::msg::Header & header,
  const geometry_msgs::msg::Vector3 & linear,
  const geometry_msgs::msg::Vector3 & angular)
{
  if (!(rviz_common::validateFloats(linear) && rviz_common::validateFloats(angular))) {
    rviz_common::MessageFilterDisplay<MessageType>::setStatus(
      rviz_common::properties::StatusProperty::Error, "Topic",
      "Message contained invalid floating point values (nans or infs)");
    return;
  }

  // Here we call the rviz::FrameManager to get the transform from the
  // fixed frame to the frame in the header of this Imu message.  If
  // it fails, we can't do anything else so we return.
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  rclcpp::Time time_stamp(header.stamp, RCL_ROS_TIME);
  if (!rviz_common::MessageFilterDisplay<MessageType>::context_->getFrameManager()->getTransform(
      header.frame_id, time_stamp, position,
      orientation))
  {
    rviz_common::MessageFilterDisplay<MessageType>::setStatus(
      rviz_common::properties::StatusProperty::Error, "Topic",
      QString("Error transforming from frame '") +
      QString::fromStdString(header.frame_id.c_str()) +
      QString("' to frame '") +
      qPrintable(rviz_common::MessageFilterDisplay<MessageType>::fixed_frame_) +
      QString("'"));
    return;
  }

  // We are keeping a circular buffer of visual pointers.
  // This gets the next one, or creates and stores it if the buffer is not full
  std::shared_ptr<rviz_rendering::ScrewVisual> visual;
  if (visuals_.size() == static_cast<size_t>(history_length_property_->getInt())) {
    visual = visuals_.front();
  } else {
    visual = std::make_shared<rviz_rendering::ScrewVisual>(
      rviz_common::MessageFilterDisplay<MessageType>::context_->getSceneManager(),
      rviz_common::MessageFilterDisplay<MessageType>::scene_node_);
  }

  if (visuals_.size() >= static_cast<size_t>(history_length_property_->getInt())) {
    visuals_.pop_front();
  }

  // Now set or update the contents of the chosen visual.
  visual->setScrew(
    Ogre::Vector3(linear.x, linear.y, linear.z),
    Ogre::Vector3(angular.x, angular.y, angular.z));
  visual->setFramePosition(position);
  visual->setFrameOrientation(orientation);
  float alpha = alpha_property_->getFloat();
  Ogre::ColourValue linear_color = linear_color_property_->getOgreColor();
  Ogre::ColourValue angular_color = angular_color_property_->getOgreColor();
  visual->setLinearColor(linear_color.r, linear_color.g, linear_color.b, alpha);
  visual->setAngularColor(angular_color.r, angular_color.g, angular_color.b, alpha);
  visual->setLinearScale(linear_scale_property_->getFloat());
  visual->setAngularScale(angular_scale_property_->getFloat());
  visual->setWidth(width_property_->getFloat());
  visual->setHideSmallValues(hide_small_values_property_->getBool());

  // And send it to the end of the circular buffer
  visuals_.push_back(visual);
}

template class ScrewDisplay<geometry_msgs::msg::TwistStamped>;
template class ScrewDisplay<geometry_msgs::msg::AccelStamped>;

}  // namespace displays
}  // namespace rviz_default_plugins
