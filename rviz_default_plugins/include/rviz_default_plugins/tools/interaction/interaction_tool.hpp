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

#ifndef RVIZ_DEFAULT_PLUGINS__TOOLS__INTERACTION__INTERACTION_TOOL_HPP_
#define RVIZ_DEFAULT_PLUGINS__TOOLS__INTERACTION__INTERACTION_TOOL_HPP_

#include <cstdint>
#include <memory>

#include "rviz_common/interactive_object.hpp"
#include "rviz_common/tool.hpp"

#include "rviz_default_plugins/tools/move/move_tool.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

namespace rviz_common
{
class RenderPanel;
class ViewportMouseEvent;

namespace properties
{
class BoolProperty;
}
}

namespace rviz_default_plugins
{
namespace tools
{

class RVIZ_DEFAULT_PLUGINS_PUBLIC InteractionTool : public rviz_common::Tool
{
  Q_OBJECT

public:
  InteractionTool();
  virtual ~InteractionTool();

  void onInitialize() override;

  void activate() override;
  void deactivate() override;

  int processMouseEvent(rviz_common::ViewportMouseEvent & event) override;
  int processKeyEvent(QKeyEvent * event, rviz_common::RenderPanel * panel) override;

public Q_SLOTS:
  void hideInactivePropertyChanged() {}

protected:
  inline bool isMouseEventDragging(const rviz_common::ViewportMouseEvent & event)
  {
    // We are dragging if a button was down and is still down
    Qt::MouseButtons buttons = event.buttons_down &
      (Qt::LeftButton | Qt::RightButton | Qt::MiddleButton);
    if (event.type == QEvent::MouseButtonPress) {
      buttons &= ~event.acting_button;
    }
    return buttons != 0;
  }

  /// Check if the mouse has moved from one object to another and update focus accordingly.
  void updateFocus(const rviz_common::ViewportMouseEvent & event);

  void processInteraction(rviz_common::ViewportMouseEvent & event, const bool dragging);

  /// The object (control) which currently has the mouse focus.
  rviz_common::InteractiveObjectWPtr focused_object_;

  uint64_t last_selection_frame_count_;

  MoveTool move_tool_;

  std::unique_ptr<rviz_common::properties::BoolProperty> hide_inactive_property_;
};

}  // namespace tools
}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__TOOLS__INTERACTION__INTERACTION_TOOL_HPP_
