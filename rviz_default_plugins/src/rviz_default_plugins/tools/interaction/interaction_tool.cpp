/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * Copyright (c) 2019, Open Source Robotics Foundation, Inc.
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

#include <memory>

#include <OgreCamera.h>
#include <OgrePlane.h>
#include <OgreRay.h>
#include <OgreSceneNode.h>
#include <OgreViewport.h>

#include "rviz_common/display_context.hpp"
#include "rviz_common/interaction/forwards.hpp"
#include "rviz_common/interaction/handler_manager_iface.hpp"
#include "rviz_common/interaction/selection_manager_iface.hpp"
#include "rviz_common/interaction/selection_handler.hpp"
#include "rviz_common/load_resource.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/view_controller.hpp"
#include "rviz_common/viewport_mouse_event.hpp"

#include "rviz_default_plugins/tools/interaction/interaction_tool.hpp"

namespace rviz_default_plugins
{
namespace tools
{

InteractionTool::InteractionTool()
{
  shortcut_key_ = 'i';
  hide_inactive_property_ = std::make_unique<rviz_common::properties::BoolProperty>(
    "Hide Inactive Objects",
    true,
    "While holding down a mouse button, hide all other Interactive Objects.",
    getPropertyContainer(),
    SLOT(hideInactivePropertyChanged()),
    this);
}

InteractionTool::~InteractionTool() = default;

void InteractionTool::onInitialize()
{
  move_tool_.initialize(context_);
  last_selection_frame_count_ = context_->getFrameCount();
  deactivate();
}

void InteractionTool::activate()
{
  context_->getHandlerManager()->enableInteraction(true);
  context_->getSelectionManager()->setTextureSize(1);
}

void InteractionTool::deactivate()
{
  context_->getHandlerManager()->enableInteraction(false);
}

void InteractionTool::updateFocus(const rviz_common::ViewportMouseEvent & event)
{
  rviz_common::interaction::M_Picked results;
  const auto selection_manager = context_->getSelectionManager();
  // Pick exactly 1 pixel
  selection_manager->pick(
    event.panel->getRenderWindow(),
    event.x,
    event.y,
    event.x + 1,
    event.y + 1,
    results);

  last_selection_frame_count_ = context_->getFrameCount();

  rviz_common::InteractiveObjectPtr new_focused_object;

  // look for a valid handle in the result.
  auto result_it = results.begin();
  if (result_it != results.end()) {
    const rviz_common::interaction::Picked pick = result_it->second;
    const auto handler = context_->getHandlerManager()->getHandler(pick.handle);
    if (pick.pixel_count > 0 && handler) {
      const rviz_common::InteractiveObjectPtr object = handler->getInteractiveObject().lock();
      if (object && object->isInteractive()) {
        new_focused_object = object;
      }
    }
  }

  // If the mouse has gone from one object to another, defocus the old
  // and focus the new.
  rviz_common::InteractiveObjectPtr new_obj = new_focused_object;
  rviz_common::InteractiveObjectPtr old_obj = focused_object_.lock();
  if (new_obj != old_obj) {
    // Only copy the event contents here, once we know we need to use
    // a modified version of it.
    rviz_common::ViewportMouseEvent event_copy = event;
    if (old_obj) {
      event_copy.type = QEvent::FocusOut;
      old_obj->handleMouseEvent(event_copy);
    }

    if (new_obj) {
      event_copy.type = QEvent::FocusIn;
      new_obj->handleMouseEvent(event_copy);
    }
  }

  focused_object_ = new_focused_object;
}

int InteractionTool::processMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  int flags = 0;

  if (event.panel->contextMenuVisible()) {
    return flags;
  }

  // make sure we let the vis. manager render at least one frame between selection updates
  const bool need_selection_update = context_->getFrameCount() > last_selection_frame_count_;

  const bool dragging = isMouseEventDragging(event);

  // unless we're dragging, check if there's a new object under the mouse
  if (need_selection_update &&
    !dragging &&
    event.type != QEvent::MouseButtonRelease)
  {
    updateFocus(event);
    flags = Render;
  }

  processInteraction(event, dragging);

  if (event.type == QEvent::MouseButtonRelease) {
    updateFocus(event);
  }

  return flags;
}

int InteractionTool::processKeyEvent(QKeyEvent * event, rviz_common::RenderPanel * panel)
{
  return move_tool_.processKeyEvent(event, panel);
}

void InteractionTool::processInteraction(
  rviz_common::ViewportMouseEvent & event,
  const bool dragging)
{
  rviz_common::InteractiveObjectPtr focused_object = focused_object_.lock();
  // Pass the mouse evenet to the interactive object
  // If there is no interactive object, then fallback to the move tool
  if (focused_object) {
    focused_object->handleMouseEvent(event);
    setCursor(focused_object->getCursor());
    // this will disable everything but the current interactive object
    if (hide_inactive_property_->getBool()) {
      context_->getHandlerManager()->enableInteraction(!dragging);
    }
  } else if (event.panel->getViewController()) {
    move_tool_.processMouseEvent(event);
    setCursor(move_tool_.getCursor());
    if (hide_inactive_property_->getBool()) {
      context_->getHandlerManager()->enableInteraction(true);
    }
  }
}

}  // namespace tools
}  // namespace rviz_default_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT(build/include_order)
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::tools::InteractionTool, rviz_common::Tool)
