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

#include "rviz_default_plugins/displays/odometry/odometry_display.hpp"

#include <memory>
#include <string>

#include "rviz_rendering/objects/arrow.hpp"
#include "rviz_rendering/objects/axes.hpp"

#include "rviz_common/logging.hpp"
#include "rviz_common/msg_conversions.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/covariance_property.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/int_property.hpp"
#include "rviz_common/validate_floats.hpp"

#include "./quaternion_helper.hpp"

namespace rviz_default_plugins
{
namespace displays
{
OdometryDisplay::OdometryDisplay(
  rviz_common::DisplayContext * display_context, Ogre::SceneNode * scene_node)
{
  setupProperties();
  context_ = display_context;
  scene_node_ = scene_node;
  scene_manager_ = context_->getSceneManager();
}

OdometryDisplay::OdometryDisplay()
{
  setupProperties();
}

void OdometryDisplay::setupProperties()
{
  position_tolerance_property_ = new rviz_common::properties::FloatProperty(
    "Position Tolerance", 0.1f,
    "Distance, in meters from the last arrow dropped, "
    "that will cause a new arrow to drop.",
    this);
  position_tolerance_property_->setMin(0);

  angle_tolerance_property_ = new rviz_common::properties::FloatProperty(
    "Angle Tolerance", 0.1f,
    "Angular distance from the last arrow dropped, "
    "that will cause a new arrow to drop.",
    this);
  angle_tolerance_property_->setMin(0);

  keep_property_ = new rviz_common::properties::IntProperty(
    "Keep", 100,
    "Number of arrows to keep before removing the oldest.  0 means keep all of them.",
    this);
  keep_property_->setMin(0);

  shape_property_ = new rviz_common::properties::EnumProperty(
    "Shape", "Arrow", "Shape to display the pose as.",
    this, SLOT(updateShapeChoice()));
  shape_property_->addOption("Arrow", ArrowShape);
  shape_property_->addOption("Axes", AxesShape);

  color_property_ = new rviz_common::properties::ColorProperty(
    "Color", QColor(255, 25, 0),
    "Color of the arrows.",
    shape_property_, SLOT(updateColorAndAlpha()), this);

  alpha_property_ = new rviz_common::properties::FloatProperty(
    "Alpha", 1, "Amount of transparency to apply to the arrow.",
    shape_property_, SLOT(updateColorAndAlpha()), this);
  alpha_property_->setMin(0);
  alpha_property_->setMax(1);

  shaft_length_property_ = new rviz_common::properties::FloatProperty(
    "Shaft Length", 1,
    "Length of the each arrow's shaft, in meters.",
    shape_property_, SLOT(updateArrowsGeometry()), this);

  shaft_radius_property_ = new rviz_common::properties::FloatProperty(
    "Shaft Radius", 0.05f,
    "Radius of the each arrow's shaft, in meters.",
    shape_property_, SLOT(updateArrowsGeometry()), this);

  head_length_property_ = new rviz_common::properties::FloatProperty(
    "Head Length", 0.3f,
    "Length of the each arrow's head, in meters.",
    shape_property_, SLOT(updateArrowsGeometry()), this);

  head_radius_property_ = new rviz_common::properties::FloatProperty(
    "Head Radius", 0.1f,
    "Radius of the each arrow's head, in meters.",
    shape_property_, SLOT(updateArrowsGeometry()), this);

  axes_length_property_ = new rviz_common::properties::FloatProperty(
    "Axes Length", 1, "Length of each axis, in meters.",
    shape_property_, SLOT(updateAxisGeometry()), this);

  axes_radius_property_ = new rviz_common::properties::FloatProperty(
    "Axes Radius", 0.1f, "Radius of each axis, in meters.",
    shape_property_, SLOT(updateAxisGeometry()), this);

  covariance_property_ = new rviz_common::properties::CovarianceProperty(
    "Covariance", true,
    "Whether or not the covariances of the messages should be shown.",
    this, SLOT(updateCovariances()));
}

OdometryDisplay::~OdometryDisplay() = default;

void OdometryDisplay::onInitialize()
{
  MFDClass::onInitialize();
  updateShapeChoice();
}

void OdometryDisplay::onEnable()
{
  MFDClass::onEnable();
  updateShapeVisibility();
}

void OdometryDisplay::clear()
{
  arrows_.clear();
  axes_.clear();
  covariances_.clear();

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

  for (const auto & arrow : arrows_) {
    arrow->setColor(red, green, blue, alpha);
  }
  queueRender();
}

void OdometryDisplay::updateArrowsGeometry()
{
  for (const auto & arrow : arrows_) {
    updateArrow(arrow);
  }
  queueRender();
}

void OdometryDisplay::updateAxisGeometry()
{
  for (const auto & axes : axes_) {
    updateAxes(axes);
  }
  queueRender();
}

void OdometryDisplay::updateCovariances()
{
  for (const auto & covariance : covariances_) {
    covariance->updateUserData(covariance_property_->getUserData());
  }
  queueRender();
}

void OdometryDisplay::updateAxes(const std::unique_ptr<rviz_rendering::Axes> & axes)
{
  axes->set(axes_length_property_->getFloat(), axes_radius_property_->getFloat());
}

void OdometryDisplay::updateArrow(const std::unique_ptr<rviz_rendering::Arrow> & arrow)
{
  arrow->set(
    shaft_length_property_->getFloat(),
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

  queueRender();
}

void OdometryDisplay::updateShapeVisibility()
{
  bool use_arrow = (shape_property_->getOptionInt() == ArrowShape);

  for (const auto & arrow : arrows_) {
    arrow->getSceneNode()->setVisible(use_arrow);
  }

  for (const auto & axes : axes_) {
    axes->getSceneNode()->setVisible(!use_arrow);
  }
}

bool validateFloats(nav_msgs::msg::Odometry msg)
{
  bool valid = true;
  valid = valid && rviz_common::validateFloats(msg.pose.pose);
  valid = valid && rviz_common::validateFloats(msg.pose.covariance);
  // msg.twist is not validated because it is never used.
  return valid;
}

bool validateQuaternion(nav_msgs::msg::Odometry msg)
{
  return std::abs(
    (msg.pose.pose.orientation.x * msg.pose.pose.orientation.x +
    msg.pose.pose.orientation.y * msg.pose.pose.orientation.y +
    msg.pose.pose.orientation.z * msg.pose.pose.orientation.z +
    msg.pose.pose.orientation.w * msg.pose.pose.orientation.w) - 1.0f) < 10e-3;
}

void OdometryDisplay::processMessage(nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  if (!messageIsValid(msg) || messageIsSimilarToPrevious(msg)) {
    return;
  }

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->transform(msg->header, msg->pose.pose, position, orientation)) {
    setMissingTransformToFixedFrame(msg->header.frame_id);
    return;
  }
  setTransformOk();

  bool use_arrow = (shape_property_->getOptionInt() == ArrowShape);
  arrows_.push_back(createAndSetArrow(position, orientation, use_arrow));
  axes_.push_back(createAndSetAxes(position, orientation, !use_arrow));
  covariances_.push_back(createAndSetCovarianceVisual(position, orientation, msg));

  last_used_message_ = msg;
  queueRender();
}

bool OdometryDisplay::messageIsValid(nav_msgs::msg::Odometry::ConstSharedPtr message)
{
  // if both invalid values and unnormalized quaternion are present, both messages are printed.
  bool message_is_valid = true;
  if (!validateFloats(*message)) {
    setStatus(
      rviz_common::properties::StatusProperty::Error, "Topic",
      "Message contained invalid floating point values (nans or infs)");
    message_is_valid = false;
  }

  if (!validateQuaternion(*message)) {
    setStatus(
      rviz_common::properties::StatusProperty::Error, "Topic",
      "Message contained unnormalized quaternion (squares of values don't add to 1)");
    message_is_valid = false;
  }

  return message_is_valid;
}

bool OdometryDisplay::messageIsSimilarToPrevious(nav_msgs::msg::Odometry::ConstSharedPtr message)
{
  if (!last_used_message_) {
    return false;
  }

  auto last_position = rviz_common::pointMsgToOgre(last_used_message_->pose.pose.position);
  auto current_position = rviz_common::pointMsgToOgre(message->pose.pose.position);

  auto last_orientation =
    rviz_common::quaternionMsgToOgre(last_used_message_->pose.pose.orientation);
  auto current_orientation = rviz_common::quaternionMsgToOgre(message->pose.pose.orientation);

  bool position_difference_is_within_tolerance =
    (last_position - current_position).length() < position_tolerance_property_->getFloat();
  bool angle_difference_is_within_tolerance =
    ogreQuaternionAngularDistance(last_orientation, current_orientation) <
    angle_tolerance_property_->getFloat();

  return position_difference_is_within_tolerance && angle_difference_is_within_tolerance;
}

std::unique_ptr<rviz_rendering::Arrow> OdometryDisplay::createAndSetArrow(
  const Ogre::Vector3 & position, const Ogre::Quaternion & orientation, bool use_arrow)
{
  QColor color = color_property_->getColor();
  float alpha = alpha_property_->getFloat();

  auto arrow = std::make_unique<rviz_rendering::Arrow>(
    scene_manager_, scene_node_->createChildSceneNode(),
    shaft_length_property_->getFloat(),
    shaft_radius_property_->getFloat(),
    head_length_property_->getFloat(),
    head_radius_property_->getFloat());
  arrow->setPosition(position);
  // Remember the arrow points in -Z direction, so rotate the orientation before display.
  arrow->setOrientation(orientation * Ogre::Quaternion(Ogre::Degree(-90), Ogre::Vector3::UNIT_Y));
  arrow->setColor(color.redF(), color.greenF(), color.blueF(), alpha);
  arrow->getSceneNode()->setVisible(use_arrow);

  return arrow;
}

std::unique_ptr<rviz_rendering::Axes> OdometryDisplay::createAndSetAxes(
  const Ogre::Vector3 & position, const Ogre::Quaternion & orientation, bool use_axes)
{
  auto axes = std::make_unique<rviz_rendering::Axes>(
    scene_manager_, scene_node_->createChildSceneNode(),
    axes_length_property_->getFloat(),
    axes_radius_property_->getFloat());
  axes->setPosition(position);
  axes->setOrientation(orientation);
  axes->getSceneNode()->setVisible(use_axes);

  return axes;
}

std::unique_ptr<rviz_rendering::CovarianceVisual> OdometryDisplay::createAndSetCovarianceVisual(
  const Ogre::Vector3 & position,
  const Ogre::Quaternion & orientation,
  nav_msgs::msg::Odometry::ConstSharedPtr message)
{
  auto covariance_visual = std::make_unique<rviz_rendering::CovarianceVisual>(
    scene_manager_, scene_node_->createChildSceneNode());
  covariance_visual->setPosition(position);
  covariance_visual->setOrientation(orientation);
  auto quaternion = message->pose.pose.orientation;
  covariance_visual->setCovariance(
    rviz_common::quaternionMsgToOgre(quaternion), message->pose.covariance);
  covariance_visual->updateUserData(covariance_property_->getUserData());

  return covariance_visual;
}

void OdometryDisplay::update(float wall_dt, float ros_dt)
{
  (void) wall_dt;
  (void) ros_dt;

  size_t keep = keep_property_->getInt();
  if (keep > 0) {
    while (arrows_.size() > keep) {
      arrows_.pop_front();
      covariances_.pop_front();
      axes_.pop_front();
    }
  }

  assert(arrows_.size() == axes_.size());
  assert(axes_.size() == covariances_.size());
}

void OdometryDisplay::reset()
{
  MFDClass::reset();
  clear();
}

}  // namespace displays
}  // namespace rviz_default_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::OdometryDisplay, rviz_common::Display)
