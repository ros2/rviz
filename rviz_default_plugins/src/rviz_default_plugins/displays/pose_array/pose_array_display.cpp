/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

#include "pose_array_display.hpp"

#include <OgreManualObject.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include "rviz_common/display_context.hpp"
#include "rviz_common/frame_manager.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/validate_floats.hpp"

#include "rviz_rendering/arrow.hpp"
#include "rviz_rendering/axes.hpp"

namespace rviz_default_plugins {
namespace displays {

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

  Ogre::Vector3 vectorRosToOgre( geometry_msgs::msg::Point const & point )
  {
    return Ogre::Vector3( point.x, point.y, point.z );
  }

  Ogre::Quaternion quaternionRosToOgre( geometry_msgs::msg::Quaternion const & quaternion )
  {
    return Ogre::Quaternion( quaternion.w, quaternion.x, quaternion.y, quaternion.z );
  }
}

PoseArrayDisplay::PoseArrayDisplay()
  : manual_object_( NULL )
{
  shape_property_ = new rviz_common::properties::EnumProperty(
    "Shape", "Arrow (Flat)", "Shape to display the pose as.", this, SLOT(updateShapeChoice()));

  arrow_color_property_ = new rviz_common::properties::ColorProperty(
    "Color", QColor(255, 25, 0), "Color to draw the arrows.",
    this, SLOT(updateArrowColor()));

  arrow_alpha_property_ = new rviz_common::properties::FloatProperty(
    "Alpha", 1, "Amount of transparency to apply to the displayed poses.",
    this, SLOT(updateArrowColor()));

  arrow2d_length_property_ = new rviz_common::properties::FloatProperty(
    "Arrow Length", 0.3, "Length of the arrows.",
    this, SLOT(updateArrow2dGeometry()));

  arrow3d_head_radius_property_ = new rviz_common::properties::FloatProperty(
    "Head Radius", 0.03, "Radius of the arrow's head, in meters.",
    this, SLOT(updateArrow3dGeometry()));

  arrow3d_head_length_property_ = new rviz_common::properties::FloatProperty(
    "Head Length", 0.07, "Length of the arrow's head, in meters.",
    this, SLOT(updateArrow3dGeometry()));

  arrow3d_shaft_radius_property_ = new rviz_common::properties::FloatProperty(
    "Shaft Radius", 0.01, "Radius of the arrow's shaft, in meters.",
    this, SLOT(updateArrow3dGeometry()));

  arrow3d_shaft_length_property_ = new rviz_common::properties::FloatProperty(
    "Shaft Length", 0.23, "Length of the arrow's shaft, in meters.",
    this, SLOT(updateArrow3dGeometry()));

  axes_length_property_ = new rviz_common::properties::FloatProperty(
    "Axes Length", 0.3, "Length of each axis, in meters.",
    this, SLOT(updateAxesGeometry()));

  axes_radius_property_ = new rviz_common::properties::FloatProperty(
    "Axes Radius", 0.01, "Radius of each axis, in meters.",
    this, SLOT(updateAxesGeometry()));

  shape_property_->addOption( "Arrow (Flat)", ShapeType::Arrow2d );
  shape_property_->addOption( "Arrow (3D)", ShapeType::Arrow3d );
  shape_property_->addOption( "Axes", ShapeType::Axes );
  arrow_alpha_property_->setMin( 0 );
  arrow_alpha_property_->setMax( 1 );
}

PoseArrayDisplay::~PoseArrayDisplay()
{
  if (initialized()) {
    scene_manager_->destroyManualObject(manual_object_);
  }
}

void PoseArrayDisplay::onInitialize()
{
  RTDClass::onInitialize();
  manual_object_ = scene_manager_->createManualObject();
  manual_object_->setDynamic( true );
  scene_node_->attachObject( manual_object_ );
  arrow_node_ = scene_node_->createChildSceneNode();
  axes_node_ = scene_node_->createChildSceneNode();
  updateShapeChoice();
}

bool validateFloats(const geometry_msgs::msg::PoseArray &msg)
{
  return rviz_common::validateFloats(msg.poses);
}

void PoseArrayDisplay::processMessage(
  const geometry_msgs::msg::PoseArray::ConstSharedPtr msg)
{
  if( !validateFloats( *msg ))
  {
    setStatus( rviz_common::properties::StatusProperty::Error, "Topic",
               "Message contained invalid floating point values (nans or infs)" );
    return;
  }

  if (!setTransform(msg->header)) {
    setStatus(rviz_common::properties::StatusProperty::Error, "Topic", "Failed to look up transform");
    return;
  }

  poses_.resize( msg->poses.size() );
  for (std::size_t i = 0; i < msg->poses.size(); ++i)
  {
    poses_[i].position = vectorRosToOgre( msg->poses[i].position );
    poses_[i].orientation = quaternionRosToOgre( msg->poses[i].orientation );
  }

  updateDisplay();
  context_->queueRender();
}

bool PoseArrayDisplay::setTransform(std_msgs::msg::Header const &header)
{
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->getTransform(header, position, orientation)) {
    RVIZ_COMMON_LOG_ERROR_STREAM(
      "Error transforming pose '" << qPrintable(getName()) << "' from frame '" << header.frame_id
        .c_str() << "' to frame '" << qPrintable(fixed_frame_) << "'");
    return false;
  }
  scene_node_->setPosition( position );
  scene_node_->setOrientation( orientation );
  return true;
}

void PoseArrayDisplay::updateArrows2d()
{
  manual_object_->clear();

  Ogre::ColourValue color = arrow_color_property_->getOgreColor();
  color.a = arrow_alpha_property_->getFloat();
  float length = arrow2d_length_property_->getFloat();
  size_t num_poses = poses_.size();
  manual_object_->estimateVertexCount( num_poses * 6 );
  manual_object_->begin( "BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST );
  for( size_t i=0; i < num_poses; ++i )
  {
    const Ogre::Vector3 & pos = poses_[i].position;
    const Ogre::Quaternion & orient = poses_[i].orientation;
    Ogre::Vector3 vertices[6];
    vertices[0] = pos; // back of arrow
    vertices[1] = pos + orient * Ogre::Vector3( length, 0, 0 ); // tip of arrow
    vertices[2] = vertices[ 1 ];
    vertices[3] = pos + orient * Ogre::Vector3( 0.75*length, 0.2*length, 0 );
    vertices[4] = vertices[ 1 ];
    vertices[5] = pos + orient * Ogre::Vector3( 0.75*length, -0.2*length, 0 );

    for( int i = 0; i < 6; ++i )
    {
      manual_object_->position( vertices[i] );
      manual_object_->colour( color );
    }
  }
  manual_object_->end();
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
    manual_object_->clear();
    axes_.clear();
    break;
  case ShapeType::Axes:
    updateAxes();
    manual_object_->clear();
    arrows3d_.clear();
    break;
  }
}

void PoseArrayDisplay::updateArrows3d()
{
  while (arrows3d_.size() < poses_.size())
    arrows3d_.push_back(makeArrow3d());
  while (arrows3d_.size() > poses_.size())
    arrows3d_.pop_back();

  Ogre::Quaternion adjust_orientation( Ogre::Degree(-90), Ogre::Vector3::UNIT_Y );
  for (std::size_t i = 0; i < poses_.size(); ++i)
  {
    arrows3d_[i]->setPosition( poses_[i].position );
    arrows3d_[i]->setOrientation( poses_[i].orientation * adjust_orientation );
  }
}

void PoseArrayDisplay::updateAxes()
{
  while (axes_.size() < poses_.size())
    axes_.push_back(makeAxes());
  while (axes_.size() > poses_.size())
    axes_.pop_back();
  for (std::size_t i = 0; i < poses_.size(); ++i)
  {
    axes_[i]->setPosition( poses_[i].position );
    axes_[i]->setOrientation( poses_[i].orientation );
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
  RTDClass::reset();
  if (manual_object_) {
    manual_object_->clear();
  }
  arrows3d_.clear();
  axes_.clear();
}

void PoseArrayDisplay::updateShapeChoice()
{
  int shape = shape_property_->getOptionInt();
  bool use_arrow2d = shape == ShapeType::Arrow2d;
  bool use_arrow3d = shape == ShapeType::Arrow3d;
  bool use_arrow = use_arrow2d || use_arrow3d;
  bool use_axes = shape == ShapeType::Axes;

  arrow_color_property_->setHidden( !use_arrow );
  arrow_alpha_property_->setHidden( !use_arrow );

  arrow2d_length_property_->setHidden(!use_arrow2d);

  arrow3d_shaft_length_property_->setHidden( !use_arrow3d );
  arrow3d_shaft_radius_property_->setHidden( !use_arrow3d );
  arrow3d_head_length_property_->setHidden( !use_arrow3d );
  arrow3d_head_radius_property_->setHidden( !use_arrow3d );

  axes_length_property_->setHidden( !use_axes );
  axes_radius_property_->setHidden( !use_axes );

  if (initialized())
    updateDisplay();
}

void PoseArrayDisplay::updateArrowColor()
{
  int shape = shape_property_->getOptionInt();
  Ogre::ColourValue color = arrow_color_property_->getOgreColor();
  color.a = arrow_alpha_property_->getFloat();

  if (shape == ShapeType::Arrow2d) {
    updateArrows2d();
  }
  else if (shape == ShapeType::Arrow3d)
  {
    for (std::size_t i = 0; i < arrows3d_.size(); ++i)
    {
      arrows3d_[i]->setColor( color );
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
  for (std::size_t i = 0; i < poses_.size(); ++i) {
    arrows3d_[i]->set(
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
  for (
    std::size_t i = 0; i < poses_.size(); ++i) {
    axes_[i]->set(
      axes_length_property_->getFloat(),
      axes_radius_property_->getFloat()
    );
  }
  context_->queueRender();
}

}  // namespace displays
}  // namespace rviz_default_plugins

//#include <pluginlib/class_list_macros.hpp>  // NOLINT
//PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::PoseArrayDisplay, rviz_common::Display)
