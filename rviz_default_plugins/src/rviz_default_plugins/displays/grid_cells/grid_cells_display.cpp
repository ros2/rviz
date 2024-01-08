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

#include "rviz_default_plugins/displays/grid_cells/grid_cells_display.hpp"

#include <memory>
#include <string>
#include <vector>

#include <OgreBillboardSet.h>
#include <OgreManualObject.h>

#include "rviz_rendering/objects/arrow.hpp"
#include "rviz_rendering/objects/point_cloud.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/parse_color.hpp"
#include "rviz_common/validate_floats.hpp"

namespace rviz_default_plugins
{
namespace displays
{

GridCellsDisplay::GridCellsDisplay(rviz_common::DisplayContext * context)
: GridCellsDisplay()
{
  context_ = context;
  scene_manager_ = context->getSceneManager();
  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
}

GridCellsDisplay::GridCellsDisplay()
: last_frame_count_(uint64_t(-1))
{
  color_property_ = new rviz_common::properties::ColorProperty(
    "Color", QColor(25, 255, 0),
    "Color of the grid cells.", this, SLOT(updateColor()));

  alpha_property_ = new rviz_common::properties::FloatProperty(
    "Alpha", 1.0f,
    "Amount of transparency to apply to the cells.",
    this, SLOT(updateAlpha()));
  alpha_property_->setMin(0);
  alpha_property_->setMax(1);
}

void GridCellsDisplay::onInitialize()
{
  MFDClass::onInitialize();

  setupCloud();
  updateAlpha();
}

void GridCellsDisplay::setupCloud()
{
  cloud_ = std::make_shared<rviz_rendering::PointCloud>();
  cloud_->setRenderMode(rviz_rendering::PointCloud::RM_TILES);
  cloud_->setCommonDirection(Ogre::Vector3::UNIT_Z);
  cloud_->setCommonUpVector(Ogre::Vector3::UNIT_Y);
  scene_node_->attachObject(cloud_.get());
}

GridCellsDisplay::~GridCellsDisplay()
{
  if (initialized()) {
    scene_node_->detachObject(cloud_.get());
  }
}

void GridCellsDisplay::updateAlpha()
{
  cloud_->setAlpha(alpha_property_->getFloat());
  context_->queueRender();
}

void GridCellsDisplay::updateColor()
{
  cloud_->setColor(rviz_common::properties::qtToOgre(color_property_->getColor()));
  context_->queueRender();
}

void GridCellsDisplay::processMessage(nav_msgs::msg::GridCells::ConstSharedPtr msg)
{
  if (context_->getFrameCount() == last_frame_count_) {
    return;
  }
  last_frame_count_ = context_->getFrameCount();

  cloud_->clearAndRemoveAllPoints();

  if (!messageIsValid(msg)) {
    return;
  }

  if (!setTransform(msg->header)) {
    return;
  }

  convertMessageToCloud(msg);
}

bool GridCellsDisplay::setTransform(std_msgs::msg::Header const & header)
{
  rclcpp::Time time_stamp(header.stamp, RCL_ROS_TIME);
  if (!updateFrame(header.frame_id, time_stamp)) {
    setMissingTransformToFixedFrame(header.frame_id, getNameStd());
    return false;
  }
  setTransformOk();

  return true;
}

bool validateFloats(const nav_msgs::msg::GridCells & msg)
{
  bool valid = true;
  valid = valid && rviz_common::validateFloats(msg.cell_width);
  valid = valid && rviz_common::validateFloats(msg.cell_height);
  valid = valid && rviz_common::validateFloats(msg.cells);
  return valid;
}

bool GridCellsDisplay::messageIsValid(nav_msgs::msg::GridCells::ConstSharedPtr msg)
{
  if (!validateFloats(*msg)) {
    setStatus(
      rviz_common::properties::StatusProperty::Error, "Topic",
      "Message contained invalid floating point values (nans or infs)");
    return false;
  }

  if (msg->cell_width == 0 || msg->cell_height == 0) {
    setStatus(
      rviz_common::properties::StatusProperty::Error, "Topic",
      "One of the Cell's dimension is zero, cells will be invisible.");
    return false;
  }

  if (msg->cells.empty()) {
    setStatus(
      rviz_common::properties::StatusProperty::Warn, "Topic",
      "Message is empty: there are no cells to be shown.");
    return false;
  }

  return true;
}

void GridCellsDisplay::convertMessageToCloud(nav_msgs::msg::GridCells::ConstSharedPtr msg)
{
  cloud_->setDimensions(msg->cell_width, msg->cell_height, 0.0f);

  Ogre::ColourValue color_int = rviz_common::properties::qtToOgre(color_property_->getColor());

  std::vector<rviz_rendering::PointCloud::Point> points;
  for (const auto & point : msg->cells) {
    rviz_rendering::PointCloud::Point rendering_point;
    rendering_point.position.x = point.x;
    rendering_point.position.y = point.y;
    rendering_point.position.z = point.z;
    rendering_point.color = color_int;
    points.push_back(rendering_point);
  }

  cloud_->addPoints(points.begin(), points.end());
}

}  // namespace displays
}  // namespace rviz_default_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::GridCellsDisplay, rviz_common::Display)
