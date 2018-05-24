/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

#include "odometry_display.hpp"

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include "rviz_rendering/objects/arrow.hpp"
#include "rviz_rendering/objects/axes.hpp"
#include "rviz_rendering/objects/covariance_visual.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/properties/covariance_property.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/int_property.hpp"
#include "rviz_common/validate_floats.hpp"

namespace rviz_default_plugins
{
namespace displays
{

OdometryDisplay::OdometryDisplay()
{
  position_tolerance_property_ = new rviz_common::properties::FloatProperty(
    "Position Tolerance", .1f,
    "Distance, in meters from the last arrow dropped, "
    "that will cause a new arrow to drop.",
    this);
  position_tolerance_property_->setMin(0);

  angle_tolerance_property_ = new rviz_common::properties::FloatProperty("Angle Tolerance", .1f,
      "Angular distance from the last arrow dropped, "
      "that will cause a new arrow to drop.",
      this);
  angle_tolerance_property_->setMin(0);

  keep_property_ = new rviz_common::properties::IntProperty("Keep", 100,
      "Number of arrows to keep before removing the oldest.  0 means keep all of them.",
      this);
  keep_property_->setMin(0);

  shape_property_ = new rviz_common::properties::EnumProperty(
    "Shape", "Arrow", "Shape to display the pose as.",
    this, SLOT(updateShapeChoice()));
  shape_property_->addOption("Arrow", ArrowShape);
  shape_property_->addOption("Axes", AxesShape);

  color_property_ = new rviz_common::properties::ColorProperty("Color", QColor(255, 25, 0),
      "Color of the arrows.",
      shape_property_, SLOT(updateColorAndAlpha()), this);

  alpha_property_ = new rviz_common::properties::FloatProperty(
    "Alpha", 1, "Amount of transparency to apply to the arrow.",
    shape_property_, SLOT(updateColorAndAlpha()), this);
  alpha_property_->setMin(0);
  alpha_property_->setMax(1);

  shaft_length_property_ = new rviz_common::properties::FloatProperty("Shaft Length", 1,
      "Length of the each arrow's shaft, in meters.",
      shape_property_, SLOT(updateArrowsGeometry()), this);

  // aleeper: default changed from 0.1 to match change in arrow.cpp
  shaft_radius_property_ = new rviz_common::properties::FloatProperty("Shaft Radius", 0.05f,
      "Radius of the each arrow's shaft, in meters.",
      shape_property_, SLOT(updateArrowsGeometry()), this);

  head_length_property_ = new rviz_common::properties::FloatProperty("Head Length", 0.3f,
      "Length of the each arrow's head, in meters.",
      shape_property_, SLOT(updateArrowsGeometry()), this);

  // aleeper: default changed from 0.2 to match change in arrow.cpp
  head_radius_property_ = new rviz_common::properties::FloatProperty("Head Radius", 0.1f,
      "Radius of the each arrow's head, in meters.",
      shape_property_, SLOT(updateArrowsGeometry()), this);

  axes_length_property_ = new rviz_common::properties::FloatProperty(
    "Axes Length", 1, "Length of each axis, in meters.",
    shape_property_, SLOT(updateAxisGeometry()), this);

  axes_radius_property_ = new rviz_common::properties::FloatProperty(
    "Axes Radius", 0.1f, "Radius of each axis, in meters.",
    shape_property_, SLOT(updateAxisGeometry()), this);

  covariance_property_ = new rviz_common::properties::CovarianceProperty("Covariance", true,
      "Whether or not the covariances of the messages should be shown.",
      this, SLOT(queueRender()));
}

OdometryDisplay::~OdometryDisplay()
{
  if (initialized()) {
    clear();
  }
}

void OdometryDisplay::onInitialize()
{
  RTDClass::onInitialize();
  updateShapeChoice();
}

void OdometryDisplay::onEnable()
{
  RTDClass::onEnable();
  updateShapeVisibility();
}

void OdometryDisplay::clear()
{
  D_Arrow::iterator it = arrows_.begin();
  D_Arrow::iterator end = arrows_.end();
  for (; it != end; ++it) {
    delete *it;
  }
  arrows_.clear();

  // covariances are stored in covariance_property_
  covariance_property_->clearVisual();

  D_Axes::iterator it_axes = axes_.begin();
  D_Axes::iterator end_axes = axes_.end();
  for (; it_axes != end_axes; ++it_axes) {
    delete *it_axes;
  }
  axes_.clear();

  if (last_used_message_) {
    last_used_message_.reset();
  }
}

void OdometryDisplay::updateColorAndAlpha()
{
  QColor color = color_property_->getColor();
  float red = color.redF();
  float green = color.greenF();
  float blue = color.blueF();
  float alpha = alpha_property_->getFloat();

  D_Arrow::iterator it = arrows_.begin();
  D_Arrow::iterator end = arrows_.end();
  for (; it != end; ++it) {
    rviz_rendering::Arrow * arrow = *it;
    arrow->setColor(red, green, blue, alpha);
  }
  context_->queueRender();
}

void OdometryDisplay::updateArrowsGeometry()
{
  D_Arrow::iterator it = arrows_.begin();
  D_Arrow::iterator end = arrows_.end();
  for (; it != end; ++it) {
    updateGeometry(*it);
  }
  context_->queueRender();
}

void OdometryDisplay::updateAxisGeometry()
{
  D_Axes::iterator it = axes_.begin();
  D_Axes::iterator end = axes_.end();
  for (; it != end; ++it) {
    updateGeometry(*it);
  }
  context_->queueRender();
}

void OdometryDisplay::updateGeometry(rviz_rendering::Axes * axes)
{
  axes->set(axes_length_property_->getFloat(),
    axes_radius_property_->getFloat());
}

void OdometryDisplay::updateGeometry(rviz_rendering::Arrow * arrow)
{
  arrow->set(shaft_length_property_->getFloat(),
    shaft_radius_property_->getFloat(),
    head_length_property_->getFloat(),
    head_radius_property_->getFloat());
}

void OdometryDisplay::updateShapeChoice()
{
  bool use_arrow = (shape_property_->getOptionInt() == ArrowShape);

  color_property_->setHidden(!use_arrow);
  alpha_property_->setHidden(!use_arrow);
  shaft_length_property_->setHidden(!use_arrow);
  shaft_radius_property_->setHidden(!use_arrow);
  head_length_property_->setHidden(!use_arrow);
  head_radius_property_->setHidden(!use_arrow);

  axes_length_property_->setHidden(use_arrow);
  axes_radius_property_->setHidden(use_arrow);

  updateShapeVisibility();

  context_->queueRender();
}

void OdometryDisplay::updateShapeVisibility()
{
  bool use_arrow = (shape_property_->getOptionInt() == ArrowShape);

  D_Arrow::iterator it = arrows_.begin();
  D_Arrow::iterator end = arrows_.end();
  for (; it != end; ++it) {
    (*it)->getSceneNode()->setVisible(use_arrow);
  }

  D_Axes::iterator it_axes = axes_.begin();
  D_Axes::iterator end_axes = axes_.end();
  for (; it_axes != end_axes; ++it_axes) {
    (*it_axes)->getSceneNode()->setVisible(!use_arrow);
  }
}

bool validateFloats(nav_msgs::msg::Odometry msg)
{
  bool valid = true;
  valid = valid && rviz_common::validateFloats(msg.pose.pose);
  valid = valid && rviz_common::validateFloats(msg.pose.covariance);
  valid = valid && rviz_common::validateFloats(msg.twist.twist);
  // TODO(Martin-Idel-SI): Can this be deleted?
  // valid = valid && rviz_common::validateFloats(msg.twist.covariance);
  return valid;
}

bool validateQuaternion(nav_msgs::msg::Odometry msg)
{
  bool valid = std::abs((msg.pose.pose.orientation.x * msg.pose.pose.orientation.x +
      msg.pose.pose.orientation.y * msg.pose.pose.orientation.y +
      msg.pose.pose.orientation.z * msg.pose.pose.orientation.z +
      msg.pose.pose.orientation.w * msg.pose.pose.orientation.w) - 1.0f) < 10e-3;
  return valid;
}

float ogreQuaterionAngularDistance(Ogre::Quaternion first, Ogre::Quaternion second)
{
  auto product = first * Ogre::Quaternion(second.w, -second.x, -second.y, -second.z);
  auto imaginary_norm = sqrt(pow(product.x, 2) + pow(product.y, 2) + pow(product.z, 2));

  return 2 * atan2(imaginary_norm, sqrt(pow(product.w, 2)));
}

void OdometryDisplay::processMessage(nav_msgs::msg::Odometry::ConstSharedPtr message)
{
  typedef rviz_common::properties::CovarianceProperty::CovarianceVisualPtr CovarianceVisualPtr;

  if (!validateFloats(*message)) {
    setStatus(rviz_common::properties::StatusProperty::Error, "Topic",
      "Message contained invalid floating point values (nans or infs)");
    return;
  }

  if (!validateQuaternion(*message)) {
    setStatus(rviz_common::properties::StatusProperty::Error, "Topic",
      "Message contained unnormalized quaternion (squares of values don't add to 1)");
    return;
  }

  if (last_used_message_) {
    Ogre::Vector3 last_position(
      last_used_message_->pose.pose.position.x,
      last_used_message_->pose.pose.position.y,
      last_used_message_->pose.pose.position.z);
    Ogre::Vector3 current_position(
      message->pose.pose.position.x,
      message->pose.pose.position.y,
      message->pose.pose.position.z);

    auto last_orientation = Ogre::Quaternion(
      last_used_message_->pose.pose.orientation.w,
      last_used_message_->pose.pose.orientation.x,
      last_used_message_->pose.pose.orientation.y,
      last_used_message_->pose.pose.orientation.z);
    auto current_orientation = Ogre::Quaternion(
      message->pose.pose.orientation.w,
      message->pose.pose.orientation.x,
      message->pose.pose.orientation.y,
      message->pose.pose.orientation.z);

    if ((last_position - current_position).length() < position_tolerance_property_->getFloat() &&
      ogreQuaterionAngularDistance(
        last_orientation, current_orientation) < angle_tolerance_property_->getFloat())
    {
      return;
    }
  }

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->transform(message->header, message->pose.pose, position,
    orientation))
  {
    RVIZ_COMMON_LOG_ERROR_STREAM("Error transforming odometry '" << getName().toStdString() <<
      "' from frame '" << message->header.frame_id << "' to frame '" <<
      fixed_frame_.toStdString() << "'");
    return;
  }

  // If we arrive here, we're good. Continue...

  // Create a scene node, and attach the arrow and the covariance to it
  rviz_rendering::Axes * axes = new rviz_rendering::Axes(
    scene_manager_, scene_node_,
    axes_length_property_->getFloat(),
    axes_radius_property_->getFloat());
  rviz_rendering::Arrow * arrow = new rviz_rendering::Arrow(
    scene_manager_, scene_node_,
    shaft_length_property_->getFloat(),
    shaft_radius_property_->getFloat(),
    head_length_property_->getFloat(),
    head_radius_property_->getFloat());
  CovarianceVisualPtr cov = covariance_property_->createAndPushBackVisual(
    scene_manager_, scene_node_);

  // Position the axes
  axes->setPosition(position);
  axes->setOrientation(orientation);

  // Position the arrow. Remember the arrow points in -Z direction,
  // so rotate the orientation before display.
  arrow->setPosition(position);
  arrow->setOrientation(orientation * Ogre::Quaternion(Ogre::Degree(-90), Ogre::Vector3::UNIT_Y));

  // Position the frame where the covariance is attached covariance
  cov->setPosition(position);
  cov->setOrientation(orientation);

  // Set up arrow color
  QColor color = color_property_->getColor();
  float alpha = alpha_property_->getFloat();
  arrow->setColor(color.redF(), color.greenF(), color.blueF(), alpha);

  // Set up the covariance based on the message
  auto quaternion = message->pose.pose.orientation;
  cov->setCovariance(Ogre::Quaternion(quaternion.w, quaternion.x, quaternion.y, quaternion.y),
    message->pose.covariance);

  // Show/Hide things based on current properties
  bool use_arrow = (shape_property_->getOptionInt() == ArrowShape);
  arrow->getSceneNode()->setVisible(use_arrow);
  axes->getSceneNode()->setVisible(!use_arrow);

  // store everything
  axes_.push_back(axes);
  arrows_.push_back(arrow);

  last_used_message_ = message;
  context_->queueRender();
}

void OdometryDisplay::update(float wall_dt, float ros_dt)
{
  (void) wall_dt;
  (void) ros_dt;

  size_t keep = keep_property_->getInt();
  if (keep > 0) {
    while (arrows_.size() > keep) {
      delete arrows_.front();
      arrows_.pop_front();

      // covariance visuals are stored into covariance_property_
      covariance_property_->popFrontVisual();

      delete axes_.front();
      axes_.pop_front();
    }
  }

  assert(arrows_.size() == axes_.size());
  assert(axes_.size() == covariance_property_->sizeVisual());
}

void OdometryDisplay::reset()
{
  RTDClass::reset();
  clear();
}
}  // namespace displays
}  // namespace rviz_default_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::OdometryDisplay, rviz_common::Display)
