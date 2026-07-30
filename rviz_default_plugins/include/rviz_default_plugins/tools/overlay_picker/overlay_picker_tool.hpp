// Copyright (c) 2014, JSK Lab
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef RVIZ_DEFAULT_PLUGINS__TOOLS__OVERLAY_PICKER__OVERLAY_PICKER_TOOL_HPP_
#define RVIZ_DEFAULT_PLUGINS__TOOLS__OVERLAY_PICKER__OVERLAY_PICKER_TOOL_HPP_

#include <QPointer>

#include "rviz_common/tool.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

namespace rviz_common
{
class Display;
class DisplayGroup;
class ViewportMouseEvent;
}  // namespace rviz_common

namespace rviz_default_plugins
{
namespace tools
{

/// Grid spacing, in pixels, that Shift-dragging an overlay snaps to.
constexpr int kShiftSnapGridSize = 20;

/// Round `value` down to the next multiple of kShiftSnapGridSize.
/**
 * Rounds towards negative infinity rather than towards zero, so that an
 * overlay dragged past the origin keeps moving in 20-pixel steps.
 */
RVIZ_DEFAULT_PLUGINS_OVERLAY_PUBLIC
int snapToGrid(int value);

class RVIZ_DEFAULT_PLUGINS_OVERLAY_PUBLIC OverlayPickerTool : public rviz_common::Tool
{
  Q_OBJECT

public:
  OverlayPickerTool();
  ~OverlayPickerTool() override;

  void activate() override;
  void deactivate() override;
  int processMouseEvent(rviz_common::ViewportMouseEvent & event) override;

private:
  rviz_common::Display * findTargetDisplay(
    rviz_common::DisplayGroup * display_group, int x, int y) const;
  void clearTarget();

  bool is_dragging_;
  QPointer<rviz_common::Display> target_display_;
  int move_offset_x_;
  int move_offset_y_;
};

}  // namespace tools
}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__TOOLS__OVERLAY_PICKER__OVERLAY_PICKER_TOOL_HPP_
