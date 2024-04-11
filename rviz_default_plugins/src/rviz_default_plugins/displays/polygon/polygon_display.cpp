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

#include "rviz_default_plugins/displays/polygon/polygon_display.hpp"

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreManualObject.h>
#include <OgreBillboardSet.h>
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>

#include <string>

#include "rviz_common/display_context.hpp"
#include "rviz_common/frame_manager_iface.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/parse_color.hpp"
#include "rviz_common/validate_floats.hpp"
#include "rviz_rendering/material_manager.hpp"

namespace rviz_default_plugins
{
namespace displays
{

PolygonDisplay::PolygonDisplay()
{
  color_property_ = new rviz_common::properties::ColorProperty(
    "Color", QColor(25, 255, 0),
    "Color to draw the polygon.", this, SLOT(queueRender()));
  alpha_property_ = new rviz_common::properties::FloatProperty(
    "Alpha", 1.0f,
    "Amount of transparency to apply to the polygon.", this, SLOT(queueRender()));
  alpha_property_->setMin(0);
  alpha_property_->setMax(1);

  static int polygon_count = 0;
  std::string material_name = "PolygonMaterial" + std::to_string(polygon_count++);
  material_ = rviz_rendering::MaterialManager::createMaterialWithNoLighting(material_name);
}

PolygonDisplay::~PolygonDisplay()
{
  if (initialized() ) {
    scene_manager_->destroyManualObject(manual_object_);
  }
}

void PolygonDisplay::onInitialize()
{
  MFDClass::onInitialize();

  manual_object_ = scene_manager_->createManualObject();
  manual_object_->setDynamic(true);
  scene_node_->attachObject(manual_object_);
}

void PolygonDisplay::reset()
{
  MFDClass::reset();
  manual_object_->clear();
}

bool validateFloats(geometry_msgs::msg::PolygonStamped::ConstSharedPtr msg)
{
  return rviz_common::validateFloats(msg->polygon.points);
}

void PolygonDisplay::processMessage(geometry_msgs::msg::PolygonStamped::ConstSharedPtr msg)
{
  if (!validateFloats(msg)) {
    setStatus(
      rviz_common::properties::StatusProperty::Error, "Topic",
      "Message contained invalid floating point values (nans or infs)");
    return;
  }

  rclcpp::Time msg_time(msg->header.stamp, RCL_ROS_TIME);
  if (!updateFrame(msg->header.frame_id, msg_time)) {
    setMissingTransformToFixedFrame(msg->header.frame_id);
    return;
  }
  setTransformOk();

  manual_object_->clear();

  Ogre::ColourValue color = rviz_common::properties::qtToOgre(color_property_->getColor());
  color.a = alpha_property_->getFloat();
  rviz_rendering::MaterialManager::enableAlphaBlending(material_, color.a);

  size_t num_points = msg->polygon.points.size();

  if (num_points > 0) {
    manual_object_->estimateVertexCount(num_points);
    manual_object_->begin(
      material_->getName(), Ogre::RenderOperation::OT_LINE_STRIP, "rviz_rendering");
    for (uint32_t i = 0; i < num_points + 1; ++i) {
      const geometry_msgs::msg::Point32 & msg_point = msg->polygon.points[i % num_points];
      manual_object_->position(msg_point.x, msg_point.y, msg_point.z);
      manual_object_->colour(color);
    }
    manual_object_->end();
  }
}

}  // namespace displays
}  // namespace rviz_default_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::PolygonDisplay, rviz_common::Display)
