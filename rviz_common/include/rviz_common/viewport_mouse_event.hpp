/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

#ifndef RVIZ_COMMON__VIEWPORT_MOUSE_EVENT_HPP_
#define RVIZ_COMMON__VIEWPORT_MOUSE_EVENT_HPP_

#include <QEvent>
#include <Qt>

#include "rviz_common/visibility_control.hpp"

class QMouseEvent;
class QWheelEvent;

namespace rviz_common
{

class RenderPanel;

class RVIZ_COMMON_PUBLIC ViewportMouseEvent
{
public:
  /// Constructor for use with a QMouseEvent.
  ViewportMouseEvent(RenderPanel * p, QMouseEvent * e, int lx, int ly);

  // Qt has a separate QWheelEvent for mouse wheel events which is not
  // a subclass of QMouseEvent, but has a lot of overlap with it.

  /// Constructor for use with a QWheelEvent.
  ViewportMouseEvent(RenderPanel * p, QWheelEvent * e, int lx, int ly);

  // Convenience functions for getting the state of the buttons and
  // modifiers at the time of the event.
  // For the button which caused a press or release event, use acting_button.
  /// Return true if the left button was used.
  bool left();
  /// Return true if the middle button was used.
  bool middle();
  /// Return true if the right button was used.
  bool right();
  /// Return true if the shift button was pressed during the event.
  bool shift();
  /// Return true if the ctrl button was pressed during the event.
  bool control();
  /// Return true if the alt button was pressed during the event.
  bool alt();

  // Convenience functions to tell if the event is a mouse-down or
  // mouse-up event and which button caused it.
  /// Return true if the left button was raised during the event.
  bool leftUp();
  /// Return true if the middle button was raised during the event.
  bool middleUp();
  /// Return true if the right button was raised during the event.
  bool rightUp();
  /// Return true if the left button was depressed during the event.
  bool leftDown();
  /// Return true if the middle button was depressed during the event.
  bool middleDown();
  /// Return true if the right button was depressed during the event.
  bool rightDown();

  RenderPanel * panel;
  QEvent::Type type;
  int device_pixel_ratio;
  int x;
  int y;
  /// Angle that the common vertical mouse wheel was rotated
  int wheel_delta;
  // The button which caused the event.  Can be Qt::NoButton (move or wheel events).
  Qt::MouseButton acting_button;
  Qt::MouseButtons buttons_down;
  Qt::KeyboardModifiers modifiers;
  int last_x;
  int last_y;
};

}  // namespace rviz_common

#endif  // RVIZ_COMMON__VIEWPORT_MOUSE_EVENT_HPP_
