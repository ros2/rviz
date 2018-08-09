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
/*
 * measure_tool.cpp
 *
 *  Created on: Aug 8, 2012
 *      Author: gossow
 */

#include "rviz_default_plugins/tools/measure/measure_tool.hpp"

#include <memory>
#include <sstream>

#include <OgreSceneNode.h>

#include "rviz_rendering/objects/line.hpp"

#include "rviz_common/display_context.hpp"
#include "rviz_common/load_resource.hpp"
#include "rviz_common/viewport_mouse_event.hpp"
#include "rviz_common/interaction/view_picker_iface.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/parse_color.hpp"

namespace rviz_default_plugins
{
namespace tools
{

MeasureTool::MeasureTool()
: is_line_started_(false), length_(-1)
{
  shortcut_key_ = 'n';

  color_property_ = new rviz_common::properties::ColorProperty(
    "Line color", Qt::darkYellow,
    "The topic on which to publish points.",
    getPropertyContainer(), SLOT(updateLineColor()), this);
}

void MeasureTool::onInitialize()
{
  line_ = std::make_shared<rviz_rendering::Line>(context_->getSceneManager());
  updateLineColor();

  std_cursor_ = rviz_common::getDefaultCursor();
  hit_cursor_ = rviz_common::makeIconCursor("package://rviz_common/icons/crosshair.svg");
}

void MeasureTool::activate()
{
  is_line_started_ = false;
}

void MeasureTool::deactivate()
{
}

int MeasureTool::processMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  Ogre::Vector3 pos;
  bool success = context_->getViewPicker()->get3DPoint(event.panel, event.x, event.y, pos);
  setCursor(success ? hit_cursor_ : std_cursor_);

  if (is_line_started_ && success) {
    line_->setPoints(start_, pos);
    length_ = (start_ - pos).length();
  }

  setStatusMessage();

  if (event.leftUp() && success) {
    processLeftButton(pos);
    return Render;
  }
  if (event.rightUp()) {
    processRightButton();
  }

  return 0;
}

void MeasureTool::updateLineColor()
{
  Ogre::ColourValue color = rviz_common::properties::qtToOgre(color_property_->getColor());
  line_->setColor(color);
}

void MeasureTool::setStatusMessage()
{
  std::stringstream ss;
  if (length_ > 0.0) {
    ss << "[Length: " << length_ << "m] ";
  }

  ss << "Click on two points to measure their distance. Right-click to reset.";
  setStatus(QString(ss.str().c_str()));
}

void MeasureTool::processLeftButton(const Ogre::Vector3 & pos)
{
  if (is_line_started_) {
    end_ = pos;
    line_->setPoints(start_, end_);
    is_line_started_ = false;
  } else {
    start_ = pos;
    is_line_started_ = true;
  }
}

void MeasureTool::processRightButton()
{
  is_line_started_ = false;
  line_->setVisible(false);
}

}  // namespace tools
}  // namespace rviz_default_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::tools::MeasureTool, rviz_common::Tool)
