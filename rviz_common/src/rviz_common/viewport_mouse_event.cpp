/*
 * Copyright (c) 2017, Open Source Robotics Foundation, Inc.
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

#include "rviz_common/viewport_mouse_event.hpp"

#include <QMouseEvent>
#include <QWheelEvent>

#include "rviz_common/render_panel.hpp"
#include "rviz_rendering/render_window.hpp"

namespace rviz_common
{

ViewportMouseEvent::ViewportMouseEvent(RenderPanel * p, QMouseEvent * e, int lx, int ly)
: panel(p),
  type(e->type()),
  device_pixel_ratio(static_cast<int>(panel->getRenderWindow()->devicePixelRatio())),
  x(e->x() * device_pixel_ratio),
  y(e->y() * device_pixel_ratio),
  wheel_delta(0),
  acting_button(e->button()),
  buttons_down(e->buttons()),
  modifiers(e->modifiers()),
  last_x(lx * device_pixel_ratio),
  last_y(ly * device_pixel_ratio)
{
}

ViewportMouseEvent::ViewportMouseEvent(RenderPanel * p, QWheelEvent * e, int lx, int ly)
: panel(p),
  type(e->type()),
  device_pixel_ratio(static_cast<int>(panel->getRenderWindow()->devicePixelRatio())),
#if (QT_VERSION >= QT_VERSION_CHECK(5, 14, 0))
  x(e->position().x() * device_pixel_ratio),
  y(e->position().y() * device_pixel_ratio),
#else
  x(e->x() * device_pixel_ratio),
  y(e->y() * device_pixel_ratio),
#endif
  wheel_delta(e->angleDelta().y()),
  acting_button(Qt::NoButton),
  buttons_down(e->buttons()),
  modifiers(e->modifiers()),
  last_x(lx * device_pixel_ratio),
  last_y(ly * device_pixel_ratio)
{
}

bool ViewportMouseEvent::left()
{
  return buttons_down & Qt::LeftButton;
}

bool ViewportMouseEvent::middle()
{
  return buttons_down & Qt::MiddleButton;
}

bool ViewportMouseEvent::right()
{
  return buttons_down & Qt::RightButton;
}

bool ViewportMouseEvent::shift()
{
  return modifiers & Qt::ShiftModifier;
}

bool ViewportMouseEvent::control()
{
  return modifiers & Qt::ControlModifier;
}

bool ViewportMouseEvent::alt()
{
  return modifiers & Qt::AltModifier;
}

bool ViewportMouseEvent::leftUp()
{
  return type == QEvent::MouseButtonRelease && acting_button == Qt::LeftButton;
}

bool ViewportMouseEvent::middleUp()
{
  return type == QEvent::MouseButtonRelease && acting_button == Qt::MiddleButton;
}

bool ViewportMouseEvent::rightUp()
{
  return type == QEvent::MouseButtonRelease && acting_button == Qt::RightButton;
}

bool ViewportMouseEvent::leftDown()
{
  return type == QEvent::MouseButtonPress && acting_button == Qt::LeftButton;
}

bool ViewportMouseEvent::middleDown()
{
  return type == QEvent::MouseButtonPress && acting_button == Qt::MiddleButton;
}

bool ViewportMouseEvent::rightDown()
{
  return type == QEvent::MouseButtonPress && acting_button == Qt::RightButton;
}

}  // namespace rviz_common
