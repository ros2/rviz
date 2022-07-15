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

#include <sstream>

#include <OgreCamera.h>
#include <OgreRay.h>
#include <OgreVector.h>
#include <OgreViewport.h>

#include "rviz_common/display_context.hpp"
#include "rviz_common/interaction/view_picker_iface.hpp"
#include "rviz_common/load_resource.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/viewport_mouse_event.hpp"
#include "rviz_common/view_controller.hpp"
#include "rviz_rendering/render_window.hpp"

#include "rviz_default_plugins/tools/focus/focus_tool.hpp"


namespace rviz_default_plugins
{
namespace tools
{

FocusTool::FocusTool()
: Tool()
{
  shortcut_key_ = 'c';
}

FocusTool::~FocusTool() = default;

void FocusTool::onInitialize()
{
  std_cursor_ = rviz_common::getDefaultCursor();
  hit_cursor_ = rviz_common::makeIconCursor("package://rviz_common/icons/crosshair.svg");
}

void FocusTool::activate()
{}

void FocusTool::deactivate()
{}

int FocusTool::processMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  int flags = 0;

  Ogre::Vector3 position;

  bool success = context_->getViewPicker()->get3DPoint(event.panel, event.x, event.y, position);
  setCursor(success ? hit_cursor_ : std_cursor_);

  if (!success) {
    computePositionForDirection(event, position);
    setStatus("<b>Left-Click:</b> Look in this direction.");
  } else {
    setStatusFrom(position);
  }

  if (event.leftUp()) {
    if (event.panel->getViewController()) {
      event.panel->getViewController()->lookAt(position);
    }
    flags |= Finished;
  }

  return flags;
}

void FocusTool::setStatusFrom(const Ogre::Vector3 & position)
{
  std::ostringstream s;
  s << "<b>Left-Click:</b> Focus on this point.";
  s.precision(3);
  s << " [" << position.x << "," << position.y << "," << position.z << "]";
  setStatus(s.str().c_str());
}

void FocusTool::computePositionForDirection(
  const rviz_common::ViewportMouseEvent & event, Ogre::Vector3 & position)
{
  auto viewport =
    rviz_rendering::RenderWindowOgreAdapter::getOgreViewport(event.panel->getRenderWindow());
  Ogre::Ray mouse_ray = viewport->getCamera()->getCameraToViewportRay(
    static_cast<float>(event.x) / static_cast<float>(viewport->getActualWidth()),
    static_cast<float>(event.y) / static_cast<float>(viewport->getActualHeight()));

  position = mouse_ray.getPoint(1.0);
}

}  // namespace tools
}  // namespace rviz_default_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::tools::FocusTool, rviz_common::Tool)
