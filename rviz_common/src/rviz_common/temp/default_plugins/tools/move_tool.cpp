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

#include "./move_tool.h"

#include "../../../display_context.hpp"
#include "../../../render_panel.hpp"
#include "../../../viewport_mouse_event.hpp"
#include "../../../selection/selection_manager.hpp"
#include "rviz_common/view_controller.hpp"
#include "../../../view_manager.hpp"
#include "../../../load_resource.hpp"

namespace rviz_common
{

MoveTool::MoveTool()
{
  shortcut_key_ = 'm';
  // this is needed as the move tool is instantiated by other tools
  setIcon( loadPixmap("package://rviz/icons/classes/MoveCamera.png") );
}

int MoveTool::processMouseEvent( ViewportMouseEvent& event )
{
  // printf("in MoveTool::processMouseEvent()\n");
  if (event.panel->getViewController())
  {
    event.panel->getViewController()->handleMouseEvent(event);
    setCursor( event.panel->getViewController()->getCursor() );
  }
  return 0;
}

int MoveTool::processKeyEvent( QKeyEvent* event, RenderPanel* panel )
{
  // printf("in MoveTool::processKeyEvent()\n");
  if( context_->getViewManager()->getCurrent() )
  {
    context_->getViewManager()->getCurrent()->handleKeyEvent( event, panel );
  }
  return Render;
}

} // namespace rviz_common

// #include <pluginlib/class_list_macros.h>
// PLUGINLIB_EXPORT_CLASS( rviz::MoveTool, rviz::Tool )
