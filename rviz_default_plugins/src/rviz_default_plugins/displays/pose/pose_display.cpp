/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

#include "rviz_default_plugins/displays/pose/pose_display.hpp"

#include <memory>

#include <OgreSceneNode.h>

#include "rviz_rendering/objects/arrow.hpp"
#include "rviz_rendering/objects/axes.hpp"
#include "rviz_rendering/objects/shape.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/frame_manager_iface.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/quaternion_property.hpp"
#include "rviz_common/properties/vector_property.hpp"
#include "rviz_common/interaction/selection_manager.hpp"
#include "rviz_common/validate_floats.hpp"
#include "rviz_default_plugins/displays/pose/pose_display_selection_handler.hpp"

namespace rviz_default_plugins
{
namespace displays
{

PoseDisplay::PoseDisplay()
: arrow_(nullptr), axes_(nullptr), pose_valid_(false)
{
  shape_property_ = new rviz_common::properties::EnumProperty(
    "Shape", "Arrow", "Shape to display the pose as.",
    this, SLOT(updateShapeChoice()));
  shape_property_->addOption("Arrow", Arrow);
  shape_property_->addOption("Axes", Axes);

  color_property_ = new rviz_common::properties::ColorProperty(
    "Color", QColor(255, 25, 0), "Color to draw the arrow.",
    this, SLOT(updateColorAndAlpha()));

  alpha_property_ = new rviz_common::properties::FloatProperty(
    "Alpha", 1, "Amount of transparency to apply to the arrow.",
    this, SLOT(updateColorAndAlpha()));
  alpha_property_->setMin(0);
  alpha_property_->setMax(1);

  shaft_length_property_ = new rviz_common::properties::FloatProperty(
    "Shaft Length", 1, "Length of the arrow's shaft, in meters.",
    this, SLOT(updateArrowGeometry()));

  shaft_radius_property_ = new rviz_common::properties::FloatProperty(
    "Shaft Radius", 0.05f, "Radius of the arrow's shaft, in meters.",
    this, SLOT(updateArrowGeometry()));

  head_length_property_ = new rviz_common::properties::FloatProperty(
    "Head Length", 0.3f, "Length of the arrow's head, in meters.",
    this, SLOT(updateArrowGeometry()));

  head_radius_property_ = new rviz_common::properties::FloatProperty(
    "Head Radius", 0.1f, "Radius of the arrow's head, in meters.",
    this, SLOT(updateArrowGeometry()));

  axes_length_property_ = new rviz_common::properties::FloatProperty(
    "Axes Length", 1, "Length of each axis, in meters.",
    this, SLOT(updateAxisGeometry()));

  axes_radius_property_ = new rviz_common::properties::FloatProperty(
    "Axes Radius", 0.1f, "Radius of each axis, in meters.",
    this, SLOT(updateAxisGeometry()));
}

void PoseDisplay::onInitialize()
{
  MFDClass::onInitialize();

  arrow_ = std::make_unique<rviz_rendering::Arrow>(
    scene_manager_, scene_node_,
    shaft_length_property_->getFloat(),
    shaft_radius_property_->getFloat(),
    head_length_property_->getFloat(),
    head_radius_property_->getFloat());
  arrow_->setDirection(Ogre::Vector3::UNIT_X);

  axes_ = std::make_unique<rviz_rendering::Axes>(
    scene_manager_, scene_node_,
    axes_length_property_->getFloat(),
    axes_radius_property_->getFloat());

  updateShapeChoice();
  updateColorAndAlpha();
}

PoseDisplay::~PoseDisplay() = default;

void PoseDisplay::onEnable()
{
  MFDClass::onEnable();
  updateShapeVisibility();
  setupSelectionHandler();
}

void PoseDisplay::setupSelectionHandler()
{
  coll_handler_ = rviz_common::interaction::createSelectionHandler
    <PoseDisplaySelectionHandler>(this, context_);
  coll_handler_->addTrackedObjects(arrow_->getSceneNode());
  coll_handler_->addTrackedObjects(axes_->getSceneNode());
}

void PoseDisplay::onDisable()
{
  MFDClass::onDisable();
  coll_handler_.reset();
}

void PoseDisplay::updateColorAndAlpha()
{
  Ogre::ColourValue color = color_property_->getOgreColor();
  color.a = alpha_property_->getFloat();

  arrow_->setColor(color);

  context_->queueRender();
}

void PoseDisplay::updateArrowGeometry()
{
  arrow_->set(
    shaft_length_property_->getFloat(),
    shaft_radius_property_->getFloat(),
    head_length_property_->getFloat(),
    head_radius_property_->getFloat());
  context_->queueRender();
}

void PoseDisplay::updateAxisGeometry()
{
  axes_->set(
    axes_length_property_->getFloat(),
    axes_radius_property_->getFloat());
  context_->queueRender();
}

void PoseDisplay::updateShapeChoice()
{
  bool use_arrow = (shape_property_->getOptionInt() == Arrow);

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

void PoseDisplay::updateShapeVisibility()
{
  if (!pose_valid_) {
    arrow_->getSceneNode()->setVisible(false);
    axes_->getSceneNode()->setVisible(false);
  } else {
    bool use_arrow = (shape_property_->getOptionInt() == Arrow);
    arrow_->getSceneNode()->setVisible(use_arrow);
    axes_->getSceneNode()->setVisible(!use_arrow);
  }
}

void PoseDisplay::processMessage(geometry_msgs::msg::PoseStamped::ConstSharedPtr message)
{
  if (!rviz_common::validateFloats(*message)) {
    setStatus(
      rviz_common::properties::StatusProperty::Error, "Topic",
      "Message contained invalid floating point values (nans or infs)");
    return;
  }

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (
    !context_->getFrameManager()->transform(
      message->header, message->pose, position, orientation))
  {
    setMissingTransformToFixedFrame(message->header.frame_id);
    return;
  }
  setTransformOk();

  pose_valid_ = true;
  updateShapeVisibility();

  scene_node_->setPosition(position);
  scene_node_->setOrientation(orientation);

  coll_handler_->setMessage(message);

  context_->queueRender();
}

void PoseDisplay::reset()
{
  MFDClass::reset();
  pose_valid_ = false;
  updateShapeVisibility();
}

}  // namespace displays
}  // namespace rviz_default_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::PoseDisplay, rviz_common::Display)
