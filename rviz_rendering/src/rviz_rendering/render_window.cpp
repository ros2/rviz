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

// This class is based on the Ogre documentation's recommendation on how to use with Qt5:
//
//   http://www.ogre3d.org/tikiwiki/tiki-index.php?page=Integrating+Ogre+into+QT5
//
// Which is in the public domain.
//
// As well as the Qt5 suggestions on how to integrate with OpenGL:
//
//   https://doc.qt.io/qt-5/qtgui-openglwindow-example.html
//
// Which is available under the three-clause BSD license:
//
//   https://doc.qt.io/qt-5/qtgui-openglwindow-openglwindow-h.html
//   https://doc.qt.io/qt-5/qtgui-openglwindow-openglwindow-cpp.html

#include <rviz_rendering/render_window.hpp>

#include <QWindow>
#include <QTimer>

#include "render_window_impl.hpp"

namespace rviz_rendering
{

RenderWindow::RenderWindow(QWindow * parent)
: QWindow(parent), impl_(new RenderWindowImpl(this))
{
  this->installEventFilter(this);
  QTimer * timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()), this, SLOT(renderNow()));
  timer->start(1000);
}

RenderWindow::~RenderWindow()
{
  delete impl_;
}

// In case any drawing surface backing stores (QRasterWindow or QOpenGLWindow)
// of Qt are supplied to this class in any way we inform Qt that they will be unused.
void
RenderWindow::render(QPainter *painter)
{
  printf("in RenderWindow::render(QPainter *)\n");
  Q_UNUSED(painter);
}

void
RenderWindow::render()
{
  printf("in RenderWindow::render()\n");
  impl_->render();
}

void
RenderWindow::renderLater()
{
  printf("in RenderWindow::renderLater()\n");
  impl_->renderLater();
}

void
RenderWindow::renderNow()
{
  printf("in RenderWindow::renderNow()\n");
  impl_->renderNow();
}

bool
RenderWindow::event(QEvent * event)
{
  printf("in RenderWindow::event(QEvent *)\n");
  switch (event->type()) {
    case QEvent::UpdateRequest:
      // m_update_pending = false;
      this->renderNow();
      return true;
    default:
      return QWindow::event(event);
  }
}

void
RenderWindow::exposeEvent(QExposeEvent * expose_event)
{
  printf("in RenderWindow::exposeEvent(QExposeEvent *)\n");
  Q_UNUSED(expose_event);

  if (this->isExposed()) {
    this->renderNow();
  }
}

bool
RenderWindow::eventFilter(QObject * target, QEvent * event)
{
  printf("in RenderWindow::eventFilter(QObject *, QEvent *)\n");
  if (target == this) {
    if (event->type() == QEvent::Resize) {
      if (this->isExposed()) {
        impl_->resize(this->width(), this->height());
      }
    }
  }

  return false;
}

}  // namespace rviz_rendering
