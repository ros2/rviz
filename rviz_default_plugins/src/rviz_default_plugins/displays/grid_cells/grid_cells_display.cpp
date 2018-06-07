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

#include "grid_cells_display.hpp"

#include <string>
#include <vector>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreManualObject.h>
#include <OgreBillboardSet.h>

#include "rviz_rendering/objects/arrow.hpp"
#include "rviz_rendering/objects/point_cloud.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/frame_manager_iface.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/parse_color.hpp"
#include "rviz_common/properties/ros_topic_property.hpp"
#include "rviz_common/validate_floats.hpp"
#include "rviz_common/display_context.hpp"

namespace rviz_default_plugins
{
namespace displays
{

GridCellsDisplay::GridCellsDisplay(rviz_common::DisplayContext * context)
: last_frame_count_(uint64_t(-1))
{
  context_ = context;
  scene_manager_ = context->getSceneManager();
  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  color_property_ = new rviz_common::properties::ColorProperty("Color", QColor(25, 255, 0),
      "Color of the grid cells.", this);

  alpha_property_ = new rviz_common::properties::FloatProperty("Alpha", 1.0f,
      "Amount of transparency to apply to the cells.",
      this, SLOT(updateAlpha()));
  alpha_property_->setMin(0);
  alpha_property_->setMax(1);
}

GridCellsDisplay::GridCellsDisplay()
: last_frame_count_(uint64_t(-1))
{
  color_property_ = new rviz_common::properties::ColorProperty("Color", QColor(25, 255, 0),
      "Color of the grid cells.", this);

  alpha_property_ = new rviz_common::properties::FloatProperty("Alpha", 1.0f,
      "Amount of transparency to apply to the cells.",
      this, SLOT(updateAlpha()));
  alpha_property_->setMin(0);
  alpha_property_->setMax(1);
}

void GridCellsDisplay::onInitialize()
{
  RTDClass::onInitialize();

  setupCloud();
  updateAlpha();
}

void GridCellsDisplay::setupCloud()
{
  cloud_ = new rviz_rendering::PointCloud();
  cloud_->setRenderMode(rviz_rendering::PointCloud::RM_TILES);
  cloud_->setCommonDirection(Ogre::Vector3::UNIT_Z);
  cloud_->setCommonUpVector(Ogre::Vector3::UNIT_Y);
  scene_node_->attachObject(cloud_);
}

GridCellsDisplay::~GridCellsDisplay()
{
  if (initialized()) {
    scene_node_->detachObject(cloud_);
    delete cloud_;
  }
}

void GridCellsDisplay::updateAlpha()
{
  cloud_->setAlpha(alpha_property_->getFloat());
  context_->queueRender();
}

bool validateFloats(const nav_msgs::msg::GridCells & msg)
{
  bool valid = true;
  valid = valid && rviz_common::validateFloats(msg.cell_width);
  valid = valid && rviz_common::validateFloats(msg.cell_height);
  valid = valid && rviz_common::validateFloats(msg.cells);
  return valid;
}

void GridCellsDisplay::processMessage(nav_msgs::msg::GridCells::ConstSharedPtr msg)
{
  if (context_->getFrameCount() == last_frame_count_) {
    return;
  }
  last_frame_count_ = context_->getFrameCount();

  cloud_->clearAndRemoveAllPoints();

  if (!validateFloats(*msg)) {
    setStatus(rviz_common::properties::StatusProperty::Error, "Topic",
      "Message contained invalid floating point values (nans or infs)");
    return;
  }

  setStatus(rviz_common::properties::StatusProperty::Ok, "Topic",
    QString::number(messages_received_) + " messages received");

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->getTransform(msg->header, position, orientation)) {
    std::string error = "Error transforming from frame '" + msg->header.frame_id +
      "' to frame '" + fixed_frame_.toStdString() + "'";
    setStatusStd(rviz_common::properties::StatusProperty::Error, "Transform", error);
    RVIZ_COMMON_LOG_DEBUG(error);
    return;
  }

  scene_node_->setPosition(position);
  scene_node_->setOrientation(orientation);

  if (msg->cell_width == 0) {
    setStatus(rviz_common::properties::StatusProperty::Error, "Topic",
      "Cell width is zero, cells will be invisible.");
    return;
  } else if (msg->cell_height == 0) {
    setStatus(rviz_common::properties::StatusProperty::Error, "Topic",
      "Cell height is zero, cells will be invisible.");
    return;
  }

  cloud_->setDimensions(msg->cell_width, msg->cell_height, 0.0f);

  Ogre::ColourValue color_int = rviz_common::properties::qtToOgre(color_property_->getColor());
  uint32_t num_points = msg->cells.size();

  typedef std::vector<rviz_rendering::PointCloud::Point> V_Point;
  V_Point points;
  points.resize(num_points);
  for (uint32_t i = 0; i < num_points; i++) {
    rviz_rendering::PointCloud::Point & current_point = points[i];
    current_point.position.x = msg->cells[i].x;
    current_point.position.y = msg->cells[i].y;
    current_point.position.z = msg->cells[i].z;
    current_point.color = color_int;
  }

  cloud_->clearAndRemoveAllPoints();

  if (!points.empty()) {
    cloud_->addPoints(points.begin(), points.end());
  }
}

void GridCellsDisplay::reset()
{
  Display::reset();
}

}  // namespace displays
}  // namespace rviz_default_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::GridCellsDisplay, rviz_common::Display)
