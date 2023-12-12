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

#include "rviz_common/render_panel.hpp"

#include <memory>
#include <string>

#include <OgreCamera.h>
#include <OgreSceneManager.h>

#include <QApplication>  // NOLINT: cpplint is unable to handle the include order here
#include <QGridLayout>  // NOLINT: cpplint is unable to handle the include order here
#include <QMenu>  // NOLINT: cpplint is unable to handle the include order here
#include <QMouseEvent>  // NOLINT: cpplint is unable to handle the include order here
#include <QTimer>  // NOLINT: cpplint is unable to handle the include order here
#include <QWidget>  // NOLINT: cpplint is unable to handle the include order here
// TODO(wjwwood): remove
#include <QDebug>  // NOLINT: cpplint is unable to handle the include order here
#include <QMetaEnum>  // NOLINT: cpplint is unable to handle the include order here
#include <QMetaObject>  // NOLINT: cpplint is unable to handle the include order here
#include <QTime>  // NOLINT: cpplint is unable to handle the include order here

#include "rviz_rendering/render_window.hpp"

// #include "./display.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/view_controller.hpp"
#include "rviz_common/viewport_mouse_event.hpp"

// #include "rviz/visualization_manager.h"
// #include "rviz/window_manager_interface.h"

namespace rviz_common
{

RenderPanel::RenderPanel(QWidget * parent)
: QWidget(parent),
  mouse_x_(0),
  mouse_y_(0),
  context_(nullptr),
  // scene_manager_(nullptr),
  view_controller_(nullptr),
  // default_camera_(0),
  context_menu_visible_(false),
  display_(nullptr),
  render_window_(new rviz_rendering::RenderWindow()),
  fake_mouse_move_event_timer_(new QTimer())
{
  setFocus(Qt::OtherFocusReason);
  render_window_container_widget_ = QWidget::createWindowContainer(render_window_, this);
  layout_ = new QGridLayout(this);
  layout_->addWidget(render_window_container_widget_);
  this->setLayout(layout_);
  render_window_->setOnRenderWindowMouseEventsCallback(
    std::bind(&RenderPanel::onRenderWindowMouseEvents, this, std::placeholders::_1));
  render_window_->setOnRenderWindowWheelEventsCallback(
    std::bind(&RenderPanel::wheelEvent, this, std::placeholders::_1));
}

RenderPanel::~RenderPanel()
{
  delete fake_mouse_move_event_timer_;
  // if (scene_manager_ && default_camera_) {
  //   scene_manager_->destroyCamera(default_camera_);
  // }
  // if (scene_manager_) {
  //   scene_manager_->removeListener(this);
  // }
}

void RenderPanel::initialize(DisplayContext * context, bool use_main_scene)
// void RenderPanel::initialize(Ogre::SceneManager * scene_manager, DisplayContext * context)
{
  context_ = context;

  if (use_main_scene) {
    Ogre::SceneManager * scene_manager = context_->getSceneManager();

    rviz_rendering::RenderWindowOgreAdapter::setSceneManager(
      render_window_, scene_manager);
    std::string camera_name;
    static int count = 0;
    camera_name = "RenderPanelCamera" + std::to_string(count++);
    auto default_camera = scene_manager->createCamera(camera_name);
    default_camera->setNearClipDistance(0.01f);

    auto camera_node = scene_manager->getRootSceneNode()->createChildSceneNode();
    camera_node->attachObject(default_camera);
    camera_node->setPosition(default_camera_pose_);
    camera_node->lookAt(Ogre::Vector3(0, 0, 0), Ogre::Node::TS_WORLD);

    rviz_rendering::RenderWindowOgreAdapter::setOgreCamera(render_window_, default_camera);
  }
  // scene_manager_ = scene_manager;
  // scene_manager_->addListener(this);

  // TODO(wjwwood) what is the purpose of this fake mouse move event?
  // connect(fake_mouse_move_event_timer_, SIGNAL(timeout()), this, SLOT(sendMouseMoveEvent()));
  // fake_mouse_move_event_timer_->start(33 /*milliseconds*/);
}

DisplayContext * RenderPanel::getManager()
{
  return context_;
}

ViewController * RenderPanel::getViewController()
{
  return view_controller_;
}

void RenderPanel::sendMouseMoveEvent()
{
  QPoint cursor_pos = QCursor::pos();
  QPoint mouse_rel_widget = mapFromGlobal(cursor_pos);
  if (rect().contains(mouse_rel_widget)) {
    bool mouse_over_this = false;
    QWidget * w = QApplication::widgetAt(cursor_pos);
    while (w) {
      if (w == this) {
        mouse_over_this = true;
        break;
      }
      w = w->parentWidget();
    }
    if (!mouse_over_this) {
      return;
    }

    QMouseEvent fake_event(QEvent::MouseMove,
      mouse_rel_widget,
      Qt::NoButton,
      QApplication::mouseButtons(),
      QApplication::keyboardModifiers());
    onRenderWindowMouseEvents(&fake_event);
  }
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

void RenderPanel::onRenderWindowMouseEvents(QMouseEvent * event)
{
  // qDebug() <<
  //   "in RenderPanel::onRenderWindowMouseEvents(): "
  //   "[" << QTime::currentTime().toString("HH:mm:ss:zzz") << "]:" <<
  //   "event->type() ==" << ToString(event->type());
  int last_x = mouse_x_;
  int last_y = mouse_y_;

  mouse_x_ = event->x();
  mouse_y_ = event->y();

  if (context_) {
    setFocus(Qt::MouseFocusReason);

    ViewportMouseEvent vme(this, event, last_x, last_y);
    context_->handleMouseEvent(vme);
    event->accept();
  }
}

// QWidget mouse events all get sent to onRenderWindowMouseEvents().
// QMouseEvent.type() distinguishes them later.

void RenderPanel::leaveEvent(QEvent * event)
{
  Q_UNUSED(event);
  setCursor(Qt::ArrowCursor);
  if (context_) {
    context_->setStatus("");
  }
}

void RenderPanel::mouseMoveEvent(QMouseEvent * event)
{
  onRenderWindowMouseEvents(event);
}

void RenderPanel::mousePressEvent(QMouseEvent * event)
{
  onRenderWindowMouseEvents(event);
}

void RenderPanel::mouseReleaseEvent(QMouseEvent * event)
{
  onRenderWindowMouseEvents(event);
}

void RenderPanel::mouseDoubleClickEvent(QMouseEvent * event)
{
  onRenderWindowMouseEvents(event);
}


void RenderPanel::wheelEvent(QWheelEvent * event)
{
  int last_x = mouse_x_;
  int last_y = mouse_y_;

#if (QT_VERSION >= QT_VERSION_CHECK(5, 14, 0))
  const QPoint rounded_position = event->position().toPoint();
  mouse_x_ = rounded_position.x();
  mouse_y_ = rounded_position.y();
#else
  mouse_x_ = event->x();
  mouse_y_ = event->y();
#endif

  if (context_) {
    setFocus(Qt::MouseFocusReason);

    // using rviz_rendering::RenderWindowOgreAdapter;
    ViewportMouseEvent vme(
      this,
      // RenderWindowOgreAdapter::getOgreViewport(render_window_),
      event, last_x, last_y);
    context_->handleMouseEvent(vme);
    event->accept();
  }
}

void RenderPanel::keyPressEvent(QKeyEvent * event)
{
  if (context_) {
    context_->handleChar(event, this);
  }
}

void RenderPanel::setViewController(ViewController * controller)
{
  view_controller_ = controller;

  using rviz_rendering::RenderWindowOgreAdapter;
  if (view_controller_) {
    RenderWindowOgreAdapter::setOgreCamera(
      render_window_,
      view_controller_->getCamera());
    view_controller_->activate();
  } else {
    RenderWindowOgreAdapter::setOgreCamera(render_window_, nullptr);
  }
}

rviz_rendering::RenderWindow * RenderPanel::getRenderWindow()
{
  return render_window_;
}

QSize
RenderPanel::sizeHint() const
{
  return QSize(320, 240);
}

void RenderPanel::resizeEvent(QResizeEvent * event)
{
  QWidget::resizeEvent(event);
  render_window_->windowMovedOrResized();
}

const Ogre::Vector3 RenderPanel::default_camera_pose_ = Ogre::Vector3(999999, 999999, 999999);

void RenderPanel::showContextMenu(std::shared_ptr<QMenu> menu)
{
  std::lock_guard<std::mutex> lock(context_menu_mutex_);
  context_menu_ = menu;
  context_menu_visible_ = true;

  QApplication::postEvent(this, new QContextMenuEvent(QContextMenuEvent::Mouse, QPoint()));
}

void RenderPanel::onContextMenuHide()
{
  context_menu_visible_ = false;
}

bool RenderPanel::contextMenuVisible()
{
  return context_menu_visible_;
}

void RenderPanel::contextMenuEvent(QContextMenuEvent * event)
{
  Q_UNUSED(event);
  std::shared_ptr<QMenu> context_menu;
  {
    std::lock_guard<std::mutex> lock(context_menu_mutex_);
    context_menu.swap(context_menu_);
  }

  if (context_menu) {
    connect(context_menu.get(), SIGNAL(aboutToHide()), this, SLOT(onContextMenuHide()));
    context_menu->exec(QCursor::pos());
  }
}
}  // namespace rviz_common
