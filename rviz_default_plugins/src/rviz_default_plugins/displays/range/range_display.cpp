// Copyright (c) 2012, Willow Garage, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#include "rviz_default_plugins/displays/range/range_display.hpp"

#include <cmath>
#include <limits>
#include <memory>

#include <OgreSceneManager.h>

#include "rclcpp/time.hpp"

#include "rviz_rendering/objects/shape.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/int_property.hpp"
#include "rviz_common/properties/parse_color.hpp"


namespace rviz_default_plugins
{
namespace displays
{

RangeDisplay::RangeDisplay(rviz_common::DisplayContext * display_context)
: RangeDisplay()
{
  context_ = display_context;
  scene_manager_ = context_->getSceneManager();
  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  updateBufferLength();
}

RangeDisplay::RangeDisplay()
{
  color_property_ = new rviz_common::properties::ColorProperty(
    "Color", Qt::white,
    "Color to draw the range.",
    this, SLOT(updateColorAndAlpha()));

  alpha_property_ = new rviz_common::properties::FloatProperty(
    "Alpha", 0.5f,
    "Amount of transparency to apply to the range.",
    this, SLOT(updateColorAndAlpha()));

  buffer_length_property_ = new rviz_common::properties::IntProperty(
    "Buffer Length", 1,
    "Number of prior measurements to display.",
    this, SLOT(updateBufferLength()));
  buffer_length_property_->setMin(1);

  show_variance_property_ = new rviz_common::properties::BoolProperty(
    "Show Variance", false,
    "Draw the +/- one standard deviation band around the measured range. "
    "A variance of 0 means unknown and is never drawn.",
    this, SLOT(updateVariance()));

  variance_color_property_ = new rviz_common::properties::ColorProperty(
    "Variance Color", Qt::white,
    "Color to draw the variance band.",
    show_variance_property_, SLOT(updateVariance()), this);

  variance_alpha_property_ = new rviz_common::properties::FloatProperty(
    "Variance Alpha", 0.2f,
    "Amount of transparency to apply to the variance band.",
    show_variance_property_, SLOT(updateVariance()), this);
}

void RangeDisplay::onInitialize()
{
  MFDClass::onInitialize();
  updateBufferLength();
  updateColorAndAlpha();
  updateVariance();
}

RangeDisplay::~RangeDisplay() = default;

void RangeDisplay::reset()
{
  MFDClass::reset();
  updateBufferLength();
}

void RangeDisplay::updateColorAndAlpha()
{
  auto color = color_property_->getOgreColor();
  float alpha = alpha_property_->getFloat();
  for (const auto & cone : cones_) {
    cone->setColor(color.r, color.g, color.b, alpha);
  }
  context_->queueRender();
}

void RangeDisplay::updateVariance()
{
  // The band's cones only exist while it is switched on, so a display left at the
  // defaults adds nothing to the scene.
  if (show_variance_property_->getBool()) {
    createVarianceBands();
  } else {
    destroyVarianceBands();
  }
  context_->queueRender();
}

void RangeDisplay::createVarianceBands()
{
  variance_bands_.resize(cones_.size());
  for (auto & band : variance_bands_) {
    createBandEdge(band.near_edge);
    createBandEdge(band.far_edge);
  }
}

void RangeDisplay::createBandEdge(std::shared_ptr<rviz_rendering::Shape> & edge)
{
  if (!edge) {
    edge = std::make_shared<rviz_rendering::Shape>(
      rviz_rendering::Shape::Cone, context_->getSceneManager(), scene_node_);
    edge->setScale(Ogre::Vector3(0, 0, 0));
  }
  const auto color = variance_color_property_->getOgreColor();
  edge->setColor(color.r, color.g, color.b, variance_alpha_property_->getFloat());
}

void RangeDisplay::destroyVarianceBands()
{
  variance_bands_.clear();
}

void RangeDisplay::updateBufferLength()
{
  int buffer_length = buffer_length_property_->getInt();
  auto color = color_property_->getOgreColor();
  cones_.resize(buffer_length);

  for (auto & cone : cones_) {
    cone.reset(
      new rviz_rendering::Shape(
        rviz_rendering::Shape::Cone, context_->getSceneManager(), scene_node_));

    cone->setScale(Ogre::Vector3(0, 0, 0));
    cone->setColor(color.r, color.g, color.b, 0);
  }

  // Bands are per measurement, so they have to be rebuilt with the buffer.
  destroyVarianceBands();
  if (show_variance_property_->getBool()) {
    createVarianceBands();
  }
}

void RangeDisplay::processMessage(const sensor_msgs::msg::Range::ConstSharedPtr msg)
{
  auto cone = cones_[messages_received_ % buffer_length_property_->getInt()];

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  rclcpp::Time time_stamp(msg->header.stamp, RCL_ROS_TIME);
  float displayed_range = getDisplayedRange(msg);
  auto pose = getPose(displayed_range);

  if (!context_->getFrameManager()->transform(
      msg->header.frame_id, time_stamp, pose, position, orientation))
  {
    setMissingTransformToFixedFrame(msg->header.frame_id);
    return;
  }
  setTransformOk();

  cone->setPosition(position);
  cone->setOrientation(orientation);

  float cone_width = 2.0f * displayed_range * tan(msg->field_of_view / 2.0f);
  Ogre::Vector3 scale(cone_width, displayed_range, cone_width);
  cone->setScale(scale);

  auto color = color_property_->getOgreColor();
  cone->setColor(color.r, color.g, color.b, alpha_property_->getFloat());

  updateVarianceBand(
    messages_received_ % buffer_length_property_->getInt(), msg, displayed_range);
}

void RangeDisplay::updateVarianceBand(
  size_t index,
  sensor_msgs::msg::Range::ConstSharedPtr msg,
  float displayed_range)
{
  if (index >= variance_bands_.size()) {
    return;
  }
  const VarianceBand & band = variance_bands_[index];

  // sensor_msgs/Range documents a variance of 0 as "variance unknown".
  if (!std::isfinite(msg->variance) || msg->variance <= 0.0f || displayed_range <= 0.0f) {
    hideBand(band);
    return;
  }

  const float sigma = std::sqrt(msg->variance);
  showBandEdge(band.near_edge, msg, displayed_range - sigma);
  showBandEdge(band.far_edge, msg, displayed_range + sigma);
}

void RangeDisplay::showBandEdge(
  const std::shared_ptr<rviz_rendering::Shape> & edge,
  sensor_msgs::msg::Range::ConstSharedPtr msg,
  float length)
{
  // A sigma wider than the measurement itself puts the near edge at or behind the
  // sensor, where there is nothing meaningful to draw.
  if (length <= 0.0f) {
    edge->setScale(Ogre::Vector3(0, 0, 0));
    return;
  }

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  const rclcpp::Time time_stamp(msg->header.stamp, RCL_ROS_TIME);

  if (!context_->getFrameManager()->transform(
      msg->header.frame_id, time_stamp, getPose(length), position, orientation))
  {
    edge->setScale(Ogre::Vector3(0, 0, 0));
    return;
  }

  edge->setPosition(position);
  edge->setOrientation(orientation);

  const float width = 2.0f * length * std::tan(msg->field_of_view / 2.0f);
  edge->setScale(Ogre::Vector3(width, length, width));
}

void RangeDisplay::hideBand(const VarianceBand & band)
{
  band.near_edge->setScale(Ogre::Vector3(0, 0, 0));
  band.far_edge->setScale(Ogre::Vector3(0, 0, 0));
}

float RangeDisplay::getDisplayedRange(sensor_msgs::msg::Range::ConstSharedPtr msg)
{
  float displayed_range = 0.0f;
  if (msg->min_range <= msg->range && msg->range <= msg->max_range) {
    displayed_range = msg->range;
  } else if (msg->min_range == msg->max_range) {  // Fixed distance ranger
    // NaNs and +Inf return false here: both of those should have 0.0 as the range
    if (msg->range < 0 && !std::isfinite(msg->range)) {
      displayed_range = msg->min_range;  // -Inf, display the detectable range
    }
  }
  return displayed_range;
}

geometry_msgs::msg::Pose RangeDisplay::getPose(float displayed_range)
{
  float fudge_factor = 0.008824f;  // fudge factor measured, must be inaccuracy of cone model.
  geometry_msgs::msg::Pose pose;

  pose.position.x = displayed_range / 2 - fudge_factor * displayed_range;
  pose.orientation.z = 0.707f;
  pose.orientation.w = 0.707f;

  return pose;
}

}  // namespace displays
}  // namespace rviz_default_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::RangeDisplay, rviz_common::Display)
