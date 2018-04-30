/*
 * Copyright (c) 2013, Willow Garage, Inc.
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

#include "./point_tool.hpp"

#include <sstream>

#include <OgreRay.h>
#include <OgreVector3.h>

#include "rviz_common/display_context.hpp"
#include "rviz_common/interaction/view_picker_iface.hpp"
#include "rviz_common/load_resource.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/string_property.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/viewport_mouse_event.hpp"
#include "rviz_common/view_controller.hpp"


namespace rviz_default_plugins
{
namespace tools
{

PointTool::PointTool()
: rviz_common::Tool(),
  node_(rclcpp::Node::make_shared("point_tool_node"))
{
  topic_property_ = new rviz_common::properties::StringProperty(
    "Topic", "/clicked_point",
    "The topic on which to publish points.",
    getPropertyContainer(), SLOT(updateTopic()), this);

  auto_deactivate_property_ = new rviz_common::properties::BoolProperty(
    "Single click", true,
    "Switch away from this tool after one click.",
    getPropertyContainer(), SLOT(updateAutoDeactivate()), this);

  updateTopic();
}

PointTool::~PointTool() = default;

void PointTool::onInitialize()
{
  hit_cursor_ = cursor_;
  std_cursor_ = rviz_common::getDefaultCursor();
  context_->addNodeToMainExecutor(node_);
}

void PointTool::activate() {}

void PointTool::deactivate() {}

void PointTool::updateTopic()
{
  publisher_ =
    node_->create_publisher<geometry_msgs::msg::PointStamped>(topic_property_->getStdString());
}

void PointTool::updateAutoDeactivate() {}

int PointTool::processMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  int flags = 0;

  Ogre::Vector3 pos;
  bool success = context_->getViewPicker()->get3DPoint(event.panel, event.x, event.y, pos);
  setCursor(success ? hit_cursor_ : std_cursor_);

  if (success) {
    std::ostringstream s;
    s << "<b>Left-Click:</b> Select this point.";
    s.precision(3);
    s << " [" << pos.x << "," << pos.y << "," << pos.z << "]";
    setStatus(s.str().c_str());

    if (event.leftUp()) {
      geometry_msgs::msg::PointStamped ps;
      ps.point.x = pos.x;
      ps.point.y = pos.y;
      ps.point.z = pos.z;
      ps.header.frame_id = context_->getFixedFrame().toStdString();
      ps.header.stamp = rclcpp::Clock().now();
      publisher_->publish(ps);

      if (auto_deactivate_property_->getBool()) {
        flags |= Finished;
      }
    }
  } else {
    setStatus("Move over an object to select the target point.");
  }

  return flags;
}

}  // namespace tools
}  // namespace rviz_default_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::tools::PointTool, rviz_common::Tool)
