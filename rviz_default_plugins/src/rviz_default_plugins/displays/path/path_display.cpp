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

#include "path_display.hpp"

#include <algorithm>
#include <vector>

#include <OgreManualObject.h>
#include <OgreBillboardSet.h>

#include "rviz_common/logging.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/int_property.hpp"
#include "rviz_common/properties/vector_property.hpp"
#include "rviz_common/validate_floats.hpp"

namespace rviz_default_plugins
{
namespace displays
{
PathDisplay::PathDisplay()
{
  style_property_ = new rviz_common::properties::EnumProperty("Line Style", "Lines",
      "The rendering operation to use to draw the grid lines.",
      this, SLOT(updateStyle()));

  style_property_->addOption("Lines", LINES);
  style_property_->addOption("Billboards", BILLBOARDS);

  line_width_property_ = new rviz_common::properties::FloatProperty("Line Width", 0.03f,
      "The width, in meters, of each path line."
      "Only works with the 'Billboards' style.",
      this, SLOT(updateLineWidth()), this);
  line_width_property_->setMin(0.001f);
  line_width_property_->hide();

  color_property_ = new rviz_common::properties::ColorProperty("Color", QColor(25, 255, 0),
      "Color to draw the path.", this);

  alpha_property_ = new rviz_common::properties::FloatProperty("Alpha", 1.0,
      "Amount of transparency to apply to the path.", this);

  buffer_length_property_ = new rviz_common::properties::IntProperty("Buffer Length", 1,
      "Number of paths to display.",
      this, SLOT(updateBufferLength()));
  buffer_length_property_->setMin(1);

  offset_property_ = new rviz_common::properties::VectorProperty("Offset", Ogre::Vector3::ZERO,
      "Allows you to offset the path from the origin of the reference frame.  In meters.",
      this, SLOT(updateOffset()));

  pose_style_property_ = new rviz_common::properties::EnumProperty("Pose Style", "None",
      "Shape to display the pose as.",
      this, SLOT(updatePoseStyle()));
  pose_style_property_->addOption("None", NONE);
  pose_style_property_->addOption("Axes", AXES);
  pose_style_property_->addOption("Arrows", ARROWS);

  pose_axes_length_property_ = new rviz_common::properties::FloatProperty("Length", 0.3f,
      "Length of the axes.",
      this, SLOT(updatePoseAxisGeometry()) );
  pose_axes_radius_property_ = new rviz_common::properties::FloatProperty("Radius", 0.03f,
      "Radius of the axes.",
      this, SLOT(updatePoseAxisGeometry()) );

  pose_arrow_color_property_ = new rviz_common::properties::ColorProperty("Pose Color",
      QColor(255, 85, 255),
      "Color to draw the poses.",
      this, SLOT(updatePoseArrowColor()));
  pose_arrow_shaft_length_property_ = new rviz_common::properties::FloatProperty("Shaft Length",
      0.1f,
      "Length of the arrow shaft.",
      this,
      SLOT(updatePoseArrowGeometry()));
  pose_arrow_head_length_property_ = new rviz_common::properties::FloatProperty("Head Length", 0.2f,
      "Length of the arrow head.",
      this,
      SLOT(updatePoseArrowGeometry()));
  pose_arrow_shaft_diameter_property_ = new rviz_common::properties::FloatProperty("Shaft Diameter",
      0.1f,
      "Diameter of the arrow shaft.",
      this,
      SLOT(updatePoseArrowGeometry()));
  pose_arrow_head_diameter_property_ = new rviz_common::properties::FloatProperty("Head Diameter",
      0.3f,
      "Diameter of the arrow head.",
      this,
      SLOT(updatePoseArrowGeometry()));
  pose_axes_length_property_->hide();
  pose_axes_radius_property_->hide();
  pose_arrow_color_property_->hide();
  pose_arrow_shaft_length_property_->hide();
  pose_arrow_head_length_property_->hide();
  pose_arrow_shaft_diameter_property_->hide();
  pose_arrow_head_diameter_property_->hide();
}

PathDisplay::~PathDisplay()
{
  destroyObjects();
  destroyPoseAxesChain();
  destroyPoseArrowChain();
}

void PathDisplay::onInitialize()
{
  RosTopicDisplay::onInitialize();
  updateBufferLength();
}

void PathDisplay::reset()
{
  RosTopicDisplay::reset();
  updateBufferLength();
}


void PathDisplay::allocateAxesVector(
  std::vector<rviz_rendering::Axes *> & axes_vect, int num)
{
  auto vector_size = static_cast<int>(axes_vect.size());
  if (num > vector_size) {
    for (int i = vector_size; i < num; i++) {
      auto * axes = new rviz_rendering::Axes(scene_manager_, scene_node_,
          pose_axes_length_property_->getFloat(),
          pose_axes_radius_property_->getFloat());
      axes_vect.push_back(axes);
    }
  } else if (num < vector_size) {
    for (int i = vector_size - 1; num <= i; i--) {
      delete axes_vect[i];
    }
    axes_vect.resize(num);
  }
}

void PathDisplay::allocateArrowVector(
  std::vector<rviz_rendering::Arrow *> & arrow_vect, int num)
{
  auto vector_size = static_cast<int>(arrow_vect.size());
  if (num > vector_size) {
    for (int i = vector_size; i < num; i++) {
      auto * arrow = new rviz_rendering::Arrow(scene_manager_, scene_node_);
      arrow_vect.push_back(arrow);
    }
  } else if (num < vector_size) {
    for (int i = vector_size - 1; num <= i; --i) {
      delete arrow_vect[i];
    }
    arrow_vect.resize(num);
  }
}

void PathDisplay::destroyPoseAxesChain()
{
  for (auto & axes_vect : axes_chain_) {
    allocateAxesVector(axes_vect, 0);
  }
  axes_chain_.resize(0);
}

void PathDisplay::destroyPoseArrowChain()
{
  for (auto & arrow_vect : arrow_chain_) {
    allocateArrowVector(arrow_vect, 0);
  }
  arrow_chain_.resize(0);
}

void PathDisplay::updateStyle()
{
  auto style = static_cast<LineStyle>(style_property_->getOptionInt());

  switch (style) {
    case LINES:
    default:
      line_width_property_->hide();
      break;

    case BILLBOARDS:
      line_width_property_->show();
      break;
  }

  updateBufferLength();
}

void PathDisplay::updateLineWidth()
{
  auto style = static_cast<LineStyle>(style_property_->getOptionInt());
  float line_width = line_width_property_->getFloat();

  if (style == BILLBOARDS) {
    for (auto billboard_line : billboard_lines_) {
      if (billboard_line) {billboard_line->setLineWidth(line_width);}
    }
  }
  context_->queueRender();
}

void PathDisplay::updateOffset()
{
  scene_node_->setPosition(offset_property_->getVector() );
  context_->queueRender();
}

void PathDisplay::updatePoseStyle()
{
  auto pose_style = static_cast<PoseStyle>(pose_style_property_->getOptionInt());
  switch (pose_style) {
    case AXES:
      pose_axes_length_property_->show();
      pose_axes_radius_property_->show();
      pose_arrow_color_property_->hide();
      pose_arrow_shaft_length_property_->hide();
      pose_arrow_head_length_property_->hide();
      pose_arrow_shaft_diameter_property_->hide();
      pose_arrow_head_diameter_property_->hide();
      break;
    case ARROWS:
      pose_axes_length_property_->hide();
      pose_axes_radius_property_->hide();
      pose_arrow_color_property_->show();
      pose_arrow_shaft_length_property_->show();
      pose_arrow_head_length_property_->show();
      pose_arrow_shaft_diameter_property_->show();
      pose_arrow_head_diameter_property_->show();
      break;
    default:
      pose_axes_length_property_->hide();
      pose_axes_radius_property_->hide();
      pose_arrow_color_property_->hide();
      pose_arrow_shaft_length_property_->hide();
      pose_arrow_head_length_property_->hide();
      pose_arrow_shaft_diameter_property_->hide();
      pose_arrow_head_diameter_property_->hide();
  }
  updateBufferLength();
}

void PathDisplay::updatePoseAxisGeometry()
{
  for (auto & axes_vect : axes_chain_) {
    for (auto & axes : axes_vect) {
      axes->set(pose_axes_length_property_->getFloat(),
        pose_axes_radius_property_->getFloat() );
    }
  }
  context_->queueRender();
}

void PathDisplay::updatePoseArrowColor()
{
  QColor color = pose_arrow_color_property_->getColor();

  for (auto & arrow_vect : arrow_chain_) {
    for (auto & arrow : arrow_vect) {
      arrow->setColor(color.redF(), color.greenF(), color.blueF(), 1.0f);
    }
  }
  context_->queueRender();
}

void PathDisplay::updatePoseArrowGeometry()
{
  for (auto & arrow_vect : arrow_chain_) {
    for (auto & arrow : arrow_vect) {
      arrow->set(pose_arrow_shaft_length_property_->getFloat(),
        pose_arrow_shaft_diameter_property_->getFloat(),
        pose_arrow_head_length_property_->getFloat(),
        pose_arrow_head_diameter_property_->getFloat());
    }
  }
  context_->queueRender();
}

void PathDisplay::destroyObjects()
{
  // Destroy all simple lines, if any
  for (size_t i = 0; i < manual_objects_.size(); i++) {
    Ogre::ManualObject * & manual_object = manual_objects_[i];
    if (manual_object) {
      manual_object->clear();
      scene_manager_->destroyManualObject(manual_object);
      manual_object = nullptr;  // ensure it doesn't get destroyed again
    }
  }

  // Destroy all billboards, if any
  for (size_t i = 0; i < billboard_lines_.size(); i++) {
    rviz_rendering::BillboardLine * & billboard_line = billboard_lines_[i];
    if (billboard_line) {
      delete billboard_line;  // also destroys the corresponding scene node
      billboard_line = nullptr;  // ensure it doesn't get destroyed again
    }
  }
}

void PathDisplay::updateBufferLength()
{
  // Delete old path objects
  destroyObjects();

  // Destroy all axes and arrows
  destroyPoseAxesChain();
  destroyPoseArrowChain();

  // Read options
  int buffer_length = buffer_length_property_->getInt();
  auto style = static_cast<LineStyle>(style_property_->getOptionInt());

  // Create new path objects
  switch (style) {
    case LINES:  // simple lines with fixed width of 1px
      manual_objects_.resize(buffer_length);
      for (size_t i = 0; i < manual_objects_.size(); i++) {
        Ogre::ManualObject * manual_object = scene_manager_->createManualObject();
        manual_object->setDynamic(true);
        scene_node_->attachObject(manual_object);

        manual_objects_[i] = manual_object;
      }
      break;

    case BILLBOARDS:  // billboards with configurable width
      billboard_lines_.resize(buffer_length);
      for (size_t i = 0; i < billboard_lines_.size(); i++) {
        auto * billboard_line = new rviz_rendering::BillboardLine(scene_manager_, scene_node_);
        billboard_lines_[i] = billboard_line;
      }
      break;
  }
  axes_chain_.resize(buffer_length);
  arrow_chain_.resize(buffer_length);
}

bool validateFloats(const nav_msgs::msg::Path & msg)
{
  bool valid = true;
  valid = valid && rviz_common::validateFloats(msg.poses);
  return valid;
}

void PathDisplay::processMessage(nav_msgs::msg::Path::ConstSharedPtr msg)
{
  // Calculate index of oldest element in cyclic buffer
  size_t bufferIndex = messages_received_ % buffer_length_property_->getInt();

  auto style = static_cast<LineStyle>(style_property_->getOptionInt());
  Ogre::ManualObject * manual_object = nullptr;
  rviz_rendering::BillboardLine * billboard_line = nullptr;

  // Delete oldest element
  switch (style) {
    case LINES:
      manual_object = manual_objects_[bufferIndex];
      manual_object->clear();
      break;

    case BILLBOARDS:
      billboard_line = billboard_lines_[bufferIndex];
      billboard_line->clear();
      break;
  }

  // Check if path contains invalid coordinate values
  if (!validateFloats(*msg)) {
    setStatus(rviz_common::properties::StatusProperty::Error, "Topic", "Message contained invalid "
      "floating point "
      "values (nans or infs)");
    return;
  }

  // Lookup transform into fixed frame
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->getTransform(msg->header, position, orientation)) {
    RVIZ_COMMON_LOG_ERROR_STREAM(
      "Error transforming from frame '" << msg->header.frame_id.c_str() << "' to frame '" <<
        qPrintable(fixed_frame_) << "'";
    );
  }

  Ogre::Matrix4 transform(orientation);
  transform.setTrans(position);

//  scene_node_->setPosition( position );
//  scene_node_->setOrientation( orientation );

  Ogre::ColourValue color = color_property_->getOgreColor();
  color.a = alpha_property_->getFloat();

  size_t num_points = msg->poses.size();
  float line_width = line_width_property_->getFloat();

  switch (style) {
    case LINES:
      manual_object->estimateVertexCount(num_points);
      manual_object->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP);
      for (size_t i = 0; i < num_points; ++i) {
        const geometry_msgs::msg::Point & pos = msg->poses[i].pose.position;
        Ogre::Vector3 xpos = transform * Ogre::Vector3(pos.x, pos.y, pos.z);
        manual_object->position(xpos.x, xpos.y, xpos.z);
        manual_object->colour(color);
      }

      manual_object->end();
      break;

    case BILLBOARDS:
      billboard_line->setNumLines(1);
      billboard_line->setMaxPointsPerLine(static_cast<uint32_t>(num_points));
      billboard_line->setLineWidth(line_width);

      for (size_t i = 0; i < num_points; ++i) {
        const geometry_msgs::msg::Point & pos = msg->poses[i].pose.position;
        Ogre::Vector3 xpos = transform * Ogre::Vector3(pos.x, pos.y, pos.z);
        billboard_line->addPoint(xpos, color);
      }

      break;
  }

  // process pose markers
  auto pose_style = static_cast<PoseStyle>(pose_style_property_->getOptionInt());
  std::vector<rviz_rendering::Arrow *> & arrow_vect = arrow_chain_[bufferIndex];
  std::vector<rviz_rendering::Axes *> & axes_vect = axes_chain_[bufferIndex];

  switch (pose_style) {
    case AXES:
      allocateAxesVector(axes_vect, num_points);
      for (size_t i = 0; i < num_points; ++i) {
        const geometry_msgs::msg::Point & pos = msg->poses[i].pose.position;
        Ogre::Vector3 xpos = transform * Ogre::Vector3(pos.x, pos.y, pos.z);
        axes_vect[i]->setPosition(xpos);
        Ogre::Quaternion orientation(msg->poses[i].pose.orientation.w,
          msg->poses[i].pose.orientation.x,
          msg->poses[i].pose.orientation.y,
          msg->poses[i].pose.orientation.z);
        axes_vect[i]->setOrientation(orientation);
      }
      break;

    case ARROWS:
      allocateArrowVector(arrow_vect, num_points);
      for (size_t i = 0; i < num_points; ++i) {
        const geometry_msgs::msg::Point & pos = msg->poses[i].pose.position;
        Ogre::Vector3 xpos = transform * Ogre::Vector3(pos.x, pos.y, pos.z);

        QColor color = pose_arrow_color_property_->getColor();
        arrow_vect[i]->setColor(color.redF(), color.greenF(), color.blueF(), 1.0f);

        arrow_vect[i]->set(pose_arrow_shaft_length_property_->getFloat(),
          pose_arrow_shaft_diameter_property_->getFloat(),
          pose_arrow_head_length_property_->getFloat(),
          pose_arrow_head_diameter_property_->getFloat());
        arrow_vect[i]->setPosition(xpos);
        Ogre::Quaternion orientation(msg->poses[i].pose.orientation.w,
          msg->poses[i].pose.orientation.x,
          msg->poses[i].pose.orientation.y,
          msg->poses[i].pose.orientation.z);

        Ogre::Vector3 dir(1, 0, 0);
        dir = orientation * dir;
        arrow_vect[i]->setDirection(dir);
      }
      break;

    default:
      break;
  }
  context_->queueRender();
}

}  // namespace displays
}  // namespace rviz_default_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::PathDisplay, rviz_common::Display)
