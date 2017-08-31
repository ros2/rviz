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

#ifndef RVIZ_RENDERING__RENDER_WINDOW_HPP_
#define RVIZ_RENDERING__RENDER_WINDOW_HPP_

#include <QObject>
#include <QWindow>

namespace rviz_rendering
{

class RenderWindowImpl;

/// QWindow on which a rviz rendering system draws.
class RenderWindow : public QWindow
{
  Q_OBJECT

public:
  explicit RenderWindow(QWindow * parent = Q_NULLPTR);
  virtual ~RenderWindow();

  virtual
  void
  render(QPainter * painter);

  virtual
  void
  render();

public slots:
  virtual
  void
  renderLater();

  virtual
  void
  renderNow();

  // Used to capture keyboard and mouse events.
  bool
  eventFilter(QObject * target, QEvent * event) override;

protected:
  // virtual
  // void
  // keyPressEvent(QKeyEvent * key_event) override;

  // virtual
  // void
  // keyReleaseEvent(QKeyEvent * key_event) override;

  // virtual
  // void
  // mouseMoveEvent(QMouseEvent * mouse_event) override;

  // virtual
  // void
  // wheelEvent(QWheelEvent * wheel_event) override;

  // virtual
  // void
  // mousePressEvent(QMouseEvent * mouse_event) override;

  // virtual
  // void
  // mouseReleaseEvent(QMouseEvent * mouse_event) override;

  void
  exposeEvent(QExposeEvent * expose_event) override;

  bool
  event(QEvent * event) override;

  RenderWindowImpl * impl_;
};

}  // namespace rviz_rendering

#endif  // RVIZ_RENDERING__RENDER_WINDOW_HPP_
