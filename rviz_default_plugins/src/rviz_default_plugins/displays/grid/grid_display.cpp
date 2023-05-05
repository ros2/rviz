/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * Copyright (c) 2017, Open Source Robotics Foundation, Inc.
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

#include "rviz_default_plugins/displays/grid/grid_display.hpp"

#include <cstdint>
#include <memory>
#include <string>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include "rviz_common/display_context.hpp"
#include "rviz_common/frame_manager_iface.hpp"
#include "rviz_common/properties/parse_color.hpp"
#include "rviz_common/properties/property.hpp"
#include "rviz_common/interaction/selection_manager.hpp"
#include "rviz_rendering/objects/grid.hpp"

namespace rviz_default_plugins
{
namespace displays
{

using rviz_common::properties::ColorProperty;
using rviz_common::properties::EnumProperty;
using rviz_common::properties::FloatProperty;
using rviz_common::properties::IntProperty;
using rviz_common::properties::qtToOgre;
using rviz_common::properties::StatusProperty;
using rviz_common::properties::TfFrameProperty;
using rviz_common::properties::VectorProperty;
using rviz_rendering::Grid;

GridDisplay::GridDisplay()
{
  frame_property_ = new TfFrameProperty(
    "Reference Frame", TfFrameProperty::FIXED_FRAME_STRING,
    "The TF frame this grid will use for its origin.",
    this, 0, true);

  cell_count_property_ = new IntProperty(
    "Plane Cell Count", 10,
    "The number of cells to draw in the plane of the grid.",
    this, SLOT(updateCellCount()));
  cell_count_property_->setMin(1);

  height_property_ = new IntProperty(
    "Normal Cell Count", 0,
    "The number of cells to draw along the normal vector of the grid. "
    " Setting to anything but 0 makes the grid 3D.",
    this, SLOT(updateHeight()));
  height_property_->setMin(0);

  cell_size_property_ = new FloatProperty(
    "Cell Size", 1.0f,
    "The length, in meters, of the side of each cell.",
    this, SLOT(updateCellSize()));
  cell_size_property_->setMin(0.0001f);

  style_property_ = new EnumProperty(
    "Line Style", "Lines",
    "The rendering operation to use to draw the grid lines.",
    this, SLOT(updateStyle()));
  style_property_->addOption("Lines", Grid::Lines);
  style_property_->addOption("Billboards", Grid::Billboards);

  line_width_property_ = new FloatProperty(
    "Line Width", 0.03f,
    "The width, in meters, of each grid line.",
    style_property_, SLOT(updateLineWidth()), this);
  line_width_property_->setMin(0.001f);
  line_width_property_->hide();

  color_property_ = new ColorProperty(
    "Color", Qt::gray,
    "The color of the grid lines.",
    this, SLOT(updateColor()));
  alpha_property_ = new FloatProperty(
    "Alpha", 0.5f,
    "The amount of transparency to apply to the grid lines.",
    this, SLOT(updateColor()));
  alpha_property_->setMin(0.0f);
  alpha_property_->setMax(1.0f);

  plane_property_ = new EnumProperty(
    "Plane", "XY",
    "The plane to draw the grid along.",
    this, SLOT(updatePlane()));
  plane_property_->addOption("XY", XY);
  plane_property_->addOption("XZ", XZ);
  plane_property_->addOption("YZ", YZ);

  offset_property_ = new VectorProperty(
    "Offset", Ogre::Vector3::ZERO,
    "Allows you to offset the grid from the origin of the reference frame.  In meters.",
    this, SLOT(updateOffset()));
}

GridDisplay::~GridDisplay() = default;

void GridDisplay::onInitialize()
{
  QColor color = color_property_->getColor();
  color.setAlphaF(alpha_property_->getFloat() );

  frame_property_->setFrameManager(context_->getFrameManager());
  grid_ = std::make_unique<Grid>(
    scene_manager_, scene_node_,
    (Grid::Style) style_property_->getOptionInt(),
    cell_count_property_->getInt(),
    cell_size_property_->getFloat(),
    line_width_property_->getFloat(),
    qtToOgre(color));

  grid_->getSceneNode()->setVisible(false);
  updatePlane();
}

void GridDisplay::update(float dt, float ros_dt)
{
  Q_UNUSED(dt);
  Q_UNUSED(ros_dt);
  QString qframe = frame_property_->getFrame();
  std::string frame = qframe.toStdString();

  if (updateFrame(frame)) {
    setTransformOk();
    grid_->getSceneNode()->setVisible(true);
  } else {
    setMissingTransformToFixedFrame(qframe.toStdString());
    grid_->getSceneNode()->setVisible(false);
  }
}

void GridDisplay::updateColor()
{
  QColor color = color_property_->getColor();
  color.setAlphaF(alpha_property_->getFloat());
  grid_->setColor(qtToOgre(color));
  context_->queueRender();
}

void GridDisplay::updateCellSize()
{
  grid_->setCellLength(cell_size_property_->getFloat());
  context_->queueRender();
}

void GridDisplay::updateCellCount()
{
  grid_->setCellCount(cell_count_property_->getInt());
  context_->queueRender();
}

void GridDisplay::updateLineWidth()
{
  grid_->setLineWidth(line_width_property_->getFloat());
  context_->queueRender();
}

void GridDisplay::updateHeight()
{
  grid_->setHeight(height_property_->getInt());
  context_->queueRender();
}

void GridDisplay::updateStyle()
{
  auto style = static_cast<Grid::Style>(style_property_->getOptionInt());
  grid_->setStyle(style);

  switch (style) {
    case Grid::Billboards:
      line_width_property_->show();
      break;
    case Grid::Lines:
    default:
      line_width_property_->hide();
      break;
  }
  context_->queueRender();
}

void GridDisplay::updateOffset()
{
  grid_->getSceneNode()->setPosition(offset_property_->getVector());
  context_->queueRender();
}

void GridDisplay::updatePlane()
{
  Ogre::Quaternion orient;
  switch ( (Plane) plane_property_->getOptionInt()) {
    case XZ:
      orient = Ogre::Quaternion(1, 0, 0, 0);
      break;
    case YZ:
      orient =
        Ogre::Quaternion(Ogre::Vector3(0, -1, 0), Ogre::Vector3(0, 0, 1), Ogre::Vector3(1, 0, 0));
      break;
    case XY:
    default:
      orient =
        Ogre::Quaternion(Ogre::Vector3(1, 0, 0), Ogre::Vector3(0, 0, -1), Ogre::Vector3(0, 1, 0));
      break;
  }
  grid_->getSceneNode()->setOrientation(orient);

  context_->queueRender();
}

}  // namespace displays
}  // namespace rviz_default_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::GridDisplay, rviz_common::Display)
