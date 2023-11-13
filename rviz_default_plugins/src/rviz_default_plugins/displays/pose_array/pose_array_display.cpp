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

#include "rviz_default_plugins/displays/pose_array/pose_array_display.hpp"

#include <memory>
#include <string>

#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>

#include "rviz_common/logging.hpp"
#include "rviz_common/msg_conversions.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/validate_floats.hpp"

#include "rviz_rendering/objects/arrow.hpp"
#include "rviz_rendering/objects/axes.hpp"

#include "rviz_default_plugins/displays/pose_array/flat_arrows_array.hpp"

namespace rviz_default_plugins
{
namespace displays
{
namespace
{
struct ShapeType
{
  enum
  {
    Arrow2d,
    Arrow3d,
    Axes,
  };
};

}  // namespace

PoseArrayDisplay::PoseArrayDisplay(
  rviz_common::DisplayContext * display_context,
  Ogre::SceneNode * scene_node)
: PoseArrayDisplay()
{
  context_ = display_context;
  scene_node_ = scene_node;
  scene_manager_ = context_->getSceneManager();

  arrows2d_ = std::make_unique<FlatArrowsArray>(scene_manager_);
  arrows2d_->createAndAttachManualObject(scene_node);
  arrow_node_ = scene_node_->createChildSceneNode();
  axes_node_ = scene_node_->createChildSceneNode();
  updateShapeChoice();
}

PoseArrayDisplay::PoseArrayDisplay()
{
  initializeProperties();

  shape_property_->addOption("Arrow (Flat)", ShapeType::Arrow2d);
  shape_property_->addOption("Arrow (3D)", ShapeType::Arrow3d);
  shape_property_->addOption("Axes", ShapeType::Axes);
  arrow_alpha_property_->setMin(0);
  arrow_alpha_property_->setMax(1);
}

void PoseArrayDisplay::initializeProperties()
{
  shape_property_ = new rviz_common::properties::EnumProperty(
    "Shape", "Arrow (Flat)", "Shape to display the pose as.", this, SLOT(updateShapeChoice()));

  arrow_color_property_ = new rviz_common::properties::ColorProperty(
    "Color", QColor(255, 25, 0), "Color to draw the arrows.", this, SLOT(updateArrowColor()));

  arrow_alpha_property_ = new rviz_common::properties::FloatProperty(
    "Alpha",
    1.0f,
    "Amount of transparency to apply to the displayed poses.",
    this,
    SLOT(updateArrowColor()));

  arrow2d_length_property_ = new rviz_common::properties::FloatProperty(
    "Arrow Length", 0.3f, "Length of the arrows.", this, SLOT(updateArrow2dGeometry()));

  arrow3d_head_radius_property_ = new rviz_common::properties::FloatProperty(
    "Head Radius",
    0.03f,
    "Radius of the arrow's head, in meters.",
    this,
    SLOT(updateArrow3dGeometry()));

  arrow3d_head_length_property_ = new rviz_common::properties::FloatProperty(
    "Head Length",
    0.07f,
    "Length of the arrow's head, in meters.",
    this,
    SLOT(updateArrow3dGeometry()));

  arrow3d_shaft_radius_property_ = new rviz_common::properties::FloatProperty(
    "Shaft Radius",
    0.01f,
    "Radius of the arrow's shaft, in meters.",
    this,
    SLOT(updateArrow3dGeometry()));

  arrow3d_shaft_length_property_ = new rviz_common::properties::FloatProperty(
    "Shaft Length",
    0.23f,
    "Length of the arrow's shaft, in meters.",
    this,
    SLOT(updateArrow3dGeometry()));

  axes_length_property_ = new rviz_common::properties::FloatProperty(
    "Axes Length",
    0.3f,
    "Length of each axis, in meters.",
    this,
    SLOT(updateAxesGeometry()));

  axes_radius_property_ = new rviz_common::properties::FloatProperty(
    "Axes Radius",
    0.01f,
    "Radius of each axis, in meters.",
    this,
    SLOT(updateAxesGeometry()));
}

PoseArrayDisplay::~PoseArrayDisplay()
{
  // because of forward declaration of arrow and axes, destructor cannot be declared in .hpp as
  // default
}

void PoseArrayDisplay::onInitialize()
{
  MFDClass::onInitialize();
  arrows2d_ = std::make_unique<FlatArrowsArray>(scene_manager_);
  arrows2d_->createAndAttachManualObject(scene_node_);
  arrow_node_ = scene_node_->createChildSceneNode();
  axes_node_ = scene_node_->createChildSceneNode();
  updateShapeChoice();
}

void PoseArrayDisplay::processMessage(const geometry_msgs::msg::PoseArray::ConstSharedPtr msg)
{
  if (!validateFloats(*msg)) {
    setStatus(
      rviz_common::properties::StatusProperty::Error,
      "Topic",
      "Message contained invalid floating point values (nans or infs)");
    return;
  }

  if (!setTransform(msg->header)) {
    return;
  }

  poses_.resize(msg->poses.size());

  for (std::size_t i = 0; i < msg->poses.size(); ++i) {
    poses_[i].position = rviz_common::pointMsgToOgre(msg->poses[i].position);
    poses_[i].orientation = rviz_common::quaternionMsgToOgre(msg->poses[i].orientation);
  }

  updateDisplay();

  context_->queueRender();
}

bool PoseArrayDisplay::validateFloats(const geometry_msgs::msg::PoseArray & msg)
{
  return rviz_common::validateFloats(msg.poses);
}

bool PoseArrayDisplay::setTransform(std_msgs::msg::Header const & header)
{
  rclcpp::Time time_stamp(header.stamp, RCL_ROS_TIME);
  if (!updateFrame(header.frame_id, time_stamp)) {
    setMissingTransformToFixedFrame(header.frame_id);
    return false;
  }
  setTransformOk();

  return true;
}

void PoseArrayDisplay::updateDisplay()
{
  int shape = shape_property_->getOptionInt();
  switch (shape) {
    case ShapeType::Arrow2d:
      updateArrows2d();
      arrows3d_.clear();
      axes_.clear();
      break;
    case ShapeType::Arrow3d:
      updateArrows3d();
      arrows2d_->clear();
      axes_.clear();
      break;
    case ShapeType::Axes:
      updateAxes();
      arrows2d_->clear();
      arrows3d_.clear();
      break;
  }
}

void PoseArrayDisplay::updateArrows2d()
{
  arrows2d_->updateManualObject(
    arrow_color_property_->getOgreColor(),
    arrow_alpha_property_->getFloat(),
    arrow2d_length_property_->getFloat(),
    poses_);
}

void PoseArrayDisplay::updateArrows3d()
{
  while (arrows3d_.size() < poses_.size()) {
    arrows3d_.push_back(makeArrow3d());
  }
  while (arrows3d_.size() > poses_.size()) {
    arrows3d_.pop_back();
  }

  Ogre::Quaternion adjust_orientation(Ogre::Degree(-90), Ogre::Vector3::UNIT_Y);
  for (std::size_t i = 0; i < poses_.size(); ++i) {
    arrows3d_[i]->setPosition(poses_[i].position);
    arrows3d_[i]->setOrientation(poses_[i].orientation * adjust_orientation);
  }
}

void PoseArrayDisplay::updateAxes()
{
  while (axes_.size() < poses_.size()) {
    axes_.push_back(makeAxes());
  }
  while (axes_.size() > poses_.size()) {
    axes_.pop_back();
  }
  for (std::size_t i = 0; i < poses_.size(); ++i) {
    axes_[i]->setPosition(poses_[i].position);
    axes_[i]->setOrientation(poses_[i].orientation);
  }
}

std::unique_ptr<rviz_rendering::Arrow> PoseArrayDisplay::makeArrow3d()
{
  Ogre::ColourValue color = arrow_color_property_->getOgreColor();
  color.a = arrow_alpha_property_->getFloat();

  auto arrow = std::make_unique<rviz_rendering::Arrow>(
    scene_manager_,
    arrow_node_,
    arrow3d_shaft_length_property_->getFloat(),
    arrow3d_shaft_radius_property_->getFloat(),
    arrow3d_head_length_property_->getFloat(),
    arrow3d_head_radius_property_->getFloat()
  );

  arrow->setColor(color);
  return arrow;
}

std::unique_ptr<rviz_rendering::Axes> PoseArrayDisplay::makeAxes()
{
  return std::make_unique<rviz_rendering::Axes>(
    scene_manager_,
    axes_node_,
    axes_length_property_->getFloat(),
    axes_radius_property_->getFloat()
  );
}

void PoseArrayDisplay::reset()
{
  MFDClass::reset();
  arrows2d_->clear();
  arrows3d_.clear();
  axes_.clear();
}

void PoseArrayDisplay::updateShapeChoice()
{
  int shape = shape_property_->getOptionInt();
  bool use_arrow2d = shape == ShapeType::Arrow2d;
  bool use_arrow3d = shape == ShapeType::Arrow3d;
  bool use_axes = shape == ShapeType::Axes;

  arrow_color_property_->setHidden(use_axes);
  arrow_alpha_property_->setHidden(use_axes);

  arrow2d_length_property_->setHidden(!use_arrow2d);

  arrow3d_shaft_length_property_->setHidden(!use_arrow3d);
  arrow3d_shaft_radius_property_->setHidden(!use_arrow3d);
  arrow3d_head_length_property_->setHidden(!use_arrow3d);
  arrow3d_head_radius_property_->setHidden(!use_arrow3d);

  axes_length_property_->setHidden(!use_axes);
  axes_radius_property_->setHidden(!use_axes);

  if (initialized()) {
    updateDisplay();
  }
}

void PoseArrayDisplay::updateArrowColor()
{
  int shape = shape_property_->getOptionInt();
  Ogre::ColourValue color = arrow_color_property_->getOgreColor();
  color.a = arrow_alpha_property_->getFloat();

  if (shape == ShapeType::Arrow2d) {
    updateArrows2d();
  } else if (shape == ShapeType::Arrow3d) {
    for (const auto & arrow : arrows3d_) {
      arrow->setColor(color);
    }
  }
  context_->queueRender();
}

void PoseArrayDisplay::updateArrow2dGeometry()
{
  updateArrows2d();
  context_->queueRender();
}

void PoseArrayDisplay::updateArrow3dGeometry()
{
  for (const auto & arrow : arrows3d_) {
    arrow->set(
      arrow3d_shaft_length_property_->getFloat(),
      arrow3d_shaft_radius_property_->getFloat(),
      arrow3d_head_length_property_->getFloat(),
      arrow3d_head_radius_property_->getFloat()
    );
  }
  context_->queueRender();
}

void PoseArrayDisplay::updateAxesGeometry()
{
  for (const auto & axis : axes_) {
    axis->set(axes_length_property_->getFloat(), axes_radius_property_->getFloat());
  }
  context_->queueRender();
}

void PoseArrayDisplay::setShape(QString shape)
{
  shape_property_->setValue(shape);
}

}  // namespace displays
}  // namespace rviz_default_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::PoseArrayDisplay, rviz_common::Display)
