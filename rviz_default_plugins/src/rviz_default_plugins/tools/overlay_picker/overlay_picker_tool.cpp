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

#include "rviz_default_plugins/tools/overlay_picker/overlay_picker_tool.hpp"

#include <cmath>
#include <cstdint>
#include <utility>

#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/display_group.hpp"
#include "rviz_common/viewport_mouse_event.hpp"
#include "rviz_default_plugins/displays/overlay_image/overlay_image_display.hpp"

namespace
{

rviz_default_plugins::displays::OverlayImageDisplay * asOverlayImageDisplay(
  rviz_common::Display * display)
{
  return dynamic_cast<rviz_default_plugins::displays::OverlayImageDisplay *>(display);
}

bool isOverlayPickTarget(rviz_common::Display * display)
{
  return asOverlayImageDisplay(display) != nullptr;
}

bool isInOverlayRegion(rviz_common::Display * display, int x, int y)
{
  auto * overlay = asOverlayImageDisplay(display);
  return overlay && overlay->isInRegion(x, y);
}

std::pair<int, int> getOverlayPosition(rviz_common::Display * display)
{
  auto * overlay = asOverlayImageDisplay(display);
  return overlay ? overlay->getPosition() : std::make_pair(0, 0);
}

void moveOverlay(rviz_common::Display * display, int dx, int dy)
{
  auto * overlay = asOverlayImageDisplay(display);
  if (overlay) {
    overlay->movePosition(dx, dy);
  }
}

void setOverlayPosition(rviz_common::Display * display, int x, int y)
{
  auto * overlay = asOverlayImageDisplay(display);
  if (overlay) {
    overlay->setPosition(x, y);
  }
}

uint16_t getOverlayZOrder(rviz_common::Display * display)
{
  auto * overlay = asOverlayImageDisplay(display);
  return overlay ? overlay->getZOrder() : 0U;
}

}  // namespace

namespace rviz_default_plugins
{
namespace tools
{

int snapToGrid(int value)
{
  // Integer division truncates towards zero, which would snap -1 to 0 instead
  // of -20 and make the overlay jump when it is dragged past the origin.
  return static_cast<int>(
    std::floor(static_cast<double>(value) / kShiftSnapGridSize) * kShiftSnapGridSize);
}

OverlayPickerTool::OverlayPickerTool()
: rviz_common::Tool(),
  is_dragging_(false),
  move_offset_x_(0),
  move_offset_y_(0)
{
  setDescription("Left-click and drag overlays to move them. Hold Shift to snap to 20-pixel grid.");
}

OverlayPickerTool::~OverlayPickerTool() = default;

void OverlayPickerTool::activate()
{
  clearTarget();
}

void OverlayPickerTool::deactivate()
{
  clearTarget();
}

int OverlayPickerTool::processMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  int flags = 0;

  if (event.leftDown()) {
    rviz_common::Display * target = nullptr;
    if (context_ && context_->getRootDisplayGroup()) {
      target = findTargetDisplay(context_->getRootDisplayGroup(), event.x, event.y);
    }

    if (target) {
      target_display_ = target;
      is_dragging_ = true;

      const auto [target_x, target_y] = getOverlayPosition(target);
      move_offset_x_ = event.x - target_x;
      move_offset_y_ = event.y - target_y;
    } else {
      clearTarget();
    }
  }

  if (!is_dragging_) {
    return flags;
  }

  if (target_display_.isNull()) {
    clearTarget();
    return flags;
  }

  int target_x = event.x - move_offset_x_;
  int target_y = event.y - move_offset_y_;

  if (event.shift()) {
    target_x = snapToGrid(target_x);
    target_y = snapToGrid(target_y);
  }

  if (event.leftUp()) {
    setOverlayPosition(target_display_.data(), target_x, target_y);
    clearTarget();
    return Render;
  }

  if (event.left()) {
    const auto [current_x, current_y] = getOverlayPosition(target_display_.data());
    moveOverlay(target_display_.data(), target_x - current_x, target_y - current_y);
    flags |= Render;
  } else {
    clearTarget();
  }

  return flags;
}

rviz_common::Display * OverlayPickerTool::findTargetDisplay(
  rviz_common::DisplayGroup * display_group, int x, int y) const
{
  if (!display_group || !display_group->isEnabled()) {
    return nullptr;
  }

  // Where overlays overlap, pick the one drawn on top rather than whichever
  // comes first in the display list, so that dragging grabs what the user sees.
  rviz_common::Display * best = nullptr;
  uint16_t best_z_order = 0U;

  for (int i = 0; i < display_group->numDisplays(); ++i) {
    rviz_common::Display * display = display_group->getDisplayAt(i);
    if (!display || !display->isEnabled()) {
      continue;
    }

    rviz_common::Display * candidate = nullptr;
    if (auto * group = dynamic_cast<rviz_common::DisplayGroup *>(display)) {
      candidate = findTargetDisplay(group, x, y);
    } else if (isOverlayPickTarget(display) && isInOverlayRegion(display, x, y)) {
      candidate = display;
    }

    if (candidate) {
      const uint16_t z_order = getOverlayZOrder(candidate);
      if (!best || z_order >= best_z_order) {
        best = candidate;
        best_z_order = z_order;
      }
    }
  }

  return best;
}

void OverlayPickerTool::clearTarget()
{
  is_dragging_ = false;
  target_display_.clear();
  move_offset_x_ = 0;
  move_offset_y_ = 0;
}

}  // namespace tools
}  // namespace rviz_default_plugins

PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::tools::OverlayPickerTool, rviz_common::Tool)
