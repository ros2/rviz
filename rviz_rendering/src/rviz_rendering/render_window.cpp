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

#include "rviz_rendering/render_window.hpp"

#include <OgreCamera.h>

#include <string>

#include <QMouseEvent>  // NOLINT
#include <QTimer>   // NOLINT
#include <QWindow>   // NOLINT
#include <QMetaEnum>   // NOLINT
#include <QDebug>   // NOLINT
#include <QTime>   // NOLINT

// Use the Ogre implementation for now.
// This header will implement the RenderWindowImpl class.
#include "./ogre_render_window_impl.hpp"

namespace rviz_rendering
{

RenderWindow::RenderWindow(QWindow * parent)
: QWindow(parent), impl_(new RenderWindowImpl(this)),
  on_mouse_events_callback_(nullptr), on_wheel_events_callback_(nullptr)
{
  this->installEventFilter(this);
}

RenderWindow::~RenderWindow()
{
  delete impl_;
}

void RenderWindow::captureScreenShot(std::string imageName)
{
  impl_->screenShot(imageName);
}

void
RenderWindow::initialize()
{
  impl_->initialize();
}

// In case any drawing surface backing stores (QRasterWindow or QOpenGLWindow)
// of Qt are supplied to this class in any way we inform Qt that they will be unused.
void
RenderWindow::render(QPainter * painter)
{
  Q_UNUSED(painter);
}

void
RenderWindow::render()
{
  impl_->render();
}

void
RenderWindow::setOnRenderWindowMouseEventsCallback(onRenderWindowMouseEventsCallback callback)
{
  on_mouse_events_callback_ = callback;
}

void
RenderWindow::setOnRenderWindowWheelEventsCallback(onRenderWindowWheelEventsCallback callback)
{
  on_wheel_events_callback_ = callback;
}

void
RenderWindow::setupSceneAfterInit(setupSceneCallback setup_scene_callback)
{
  impl_->setupSceneAfterInit(setup_scene_callback);
}

void RenderWindow::windowMovedOrResized()
{
  // It seems that the 'width' and 'height' parameters of the resize() method don't play a role here
  impl_->resize(0, 0);
}

void
RenderWindow::renderLater()
{
  impl_->renderLater();
}

void
RenderWindow::renderNow()
{
  impl_->renderNow();
}

template<typename EnumType>
QString
ToString(const EnumType & enumValue)
{
  const char * enumName = qt_getEnumName(enumValue);
  const QMetaObject * metaObject = qt_getEnumMetaObject(enumValue);
  if (metaObject) {
    const int enumIndex = metaObject->indexOfEnumerator(enumName);
    return QString("%1::%2::%3").arg(
      metaObject->className(),
      enumName,
      metaObject->enumerator(enumIndex).valueToKey(enumValue));
  }

  return QString("%1::%2").arg(enumName).arg(static_cast<int>(enumValue));
}

bool
RenderWindow::event(QEvent * event)
{
  switch (event->type()) {
    case QEvent::Resize:
      if (this->isExposed()) {
        impl_->resize(this->width(), this->height());
      }
      return QWindow::event(event);
    case QEvent::UpdateRequest:
      this->renderNow();
      return true;
    case QEvent::Type::MouseMove:
    case QEvent::Type::MouseButtonPress:
    case QEvent::Type::MouseButtonRelease:
      if (on_mouse_events_callback_) {
        on_mouse_events_callback_(static_cast<QMouseEvent *>(event));
      }
      return QWindow::event(event);
    case QEvent::Type::Wheel:
      if (on_wheel_events_callback_) {
        on_wheel_events_callback_(static_cast<QWheelEvent *>(event));
      }
      return QWindow::event(event);
    default:
      QWindow::event(event);
      return false;
  }
}

void
RenderWindow::exposeEvent(QExposeEvent * expose_event)
{
  Q_UNUSED(expose_event);

  if (this->isExposed()) {
    impl_->resize(this->width(), this->height());
    this->renderNow();
  }
}


void
RenderWindowOgreAdapter::setOgreCamera(RenderWindow * render_window, Ogre::Camera * ogre_camera)
{
  render_window->impl_->setCamera(ogre_camera);
}

Ogre::Camera *
RenderWindowOgreAdapter::getOgreCamera(RenderWindow * render_window)
{
  return render_window->impl_->getCamera();
}

void
RenderWindowOgreAdapter::setOgreCameraPosition(
  RenderWindow * render_window,
  const Ogre::Vector3 & vec)
{
  render_window->impl_->setCameraPosition(vec);
}

void
RenderWindowOgreAdapter::setOgreCameraOrientation(
  RenderWindow * render_window,
  const Ogre::Quaternion & quat)
{
  render_window->impl_->setCameraOrientation(quat);
}

Ogre::Viewport *
RenderWindowOgreAdapter::getOgreViewport(RenderWindow * render_window)
{
  return render_window ? render_window->impl_->getViewport() : nullptr;
}

void
RenderWindowOgreAdapter::setBackgroundColor(
  RenderWindow * render_window,
  const Ogre::ColourValue * color)
{
  render_window->impl_->setBackgroundColor(*color);
}

void
RenderWindowOgreAdapter::setDirectionalLightDirection(
  RenderWindow * render_window,
  const Ogre::Vector3 & vec)
{
  return render_window->impl_->setDirectionalLightDirection(vec);
}

Ogre::Light *
RenderWindowOgreAdapter::getDirectionalLight(RenderWindow * render_window)
{
  return render_window->impl_->getDirectionalLight();
}

Ogre::SceneManager *
RenderWindowOgreAdapter::getSceneManager(RenderWindow * render_window)
{
  return render_window->impl_->getSceneManager();
}

void
RenderWindowOgreAdapter::setSceneManager(
  RenderWindow * render_window, Ogre::SceneManager * scene_manager)
{
  render_window->impl_->setSceneManager(scene_manager);
}

void RenderWindowOgreAdapter::addListener(
  RenderWindow * render_window, Ogre::RenderTargetListener * listener)
{
  render_window->impl_->addListener(listener);
}

void RenderWindowOgreAdapter::removeListener(
  RenderWindow * render_window, Ogre::RenderTargetListener * listener)
{
  render_window->impl_->removeListener(listener);
}

void RenderWindowOgreAdapter::setVisibilityMask(
  RenderWindow * render_window, uint32_t mask
)
{
  render_window->impl_->setVisibilityMask(mask);
}

}  // namespace rviz_rendering
