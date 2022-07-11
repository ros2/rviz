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

#include <functional>
#include <string>

#include <QObject>  // NOLINT
#include <QWindow>  // NOLINT

#include "OgreSceneNode.h"
#include "OgreRenderTargetListener.h"

#include "rviz_rendering/visibility_control.hpp"

// TODO(wjwwood): remove this when the camera can be abstracted
namespace Ogre
{

class Camera;
class ColourValue;
class Light;
class SceneManager;
class Viewport;

}  // namespace Ogre

namespace rviz_rendering
{

class RenderWindowImpl;
class RenderWindowOgreAdapter;

/// QWindow on which a rviz rendering system draws.
class RVIZ_RENDERING_PUBLIC RenderWindow : public QWindow
{
  Q_OBJECT

public:
  friend RenderWindowOgreAdapter;

  explicit RenderWindow(QWindow * parent = Q_NULLPTR);
  virtual ~RenderWindow();

  void
  captureScreenShot(std::string imageName);

  /// Call after adding this class to a layout.
  virtual
  void
  initialize();

  virtual
  void
  render(QPainter * painter);

  virtual
  void
  render();

  using onRenderWindowMouseEventsCallback = std::function<void (QMouseEvent * event)>;
  void
  setOnRenderWindowMouseEventsCallback(onRenderWindowMouseEventsCallback callback);

  using onRenderWindowWheelEventsCallback = std::function<void (QWheelEvent * event)>;
  void
  setOnRenderWindowWheelEventsCallback(onRenderWindowWheelEventsCallback callback);

  using setupSceneCallback = std::function<void (Ogre::SceneNode *)>;
  void
  setupSceneAfterInit(setupSceneCallback setup_scene_callback);

  void
  windowMovedOrResized();

public slots:
  virtual
  void
  renderLater();

  virtual
  void
  renderNow();

  // Used to capture keyboard and mouse events.
  // bool
  // eventFilter(QObject * target, QEvent * event) override;

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
  onRenderWindowMouseEventsCallback on_mouse_events_callback_;
  onRenderWindowWheelEventsCallback on_wheel_events_callback_;
};

// TODO(wjwwood): remove this when the Ogre stuff can be abstracted
class RVIZ_RENDERING_PUBLIC RenderWindowOgreAdapter
{
public:
  static
  void
  setOgreCamera(RenderWindow * render_window, Ogre::Camera * ogre_camera);

  static
  Ogre::Camera *
  getOgreCamera(RenderWindow * render_window);

  static
  void
  setOgreCameraPosition(RenderWindow * render_window, const Ogre::Vector3 & vec);

  static
  void
  setOgreCameraOrientation(RenderWindow * render_window, const Ogre::Quaternion & quat);

  static
  Ogre::Viewport *
  getOgreViewport(RenderWindow * render_window);

  static
  void
  setBackgroundColor(RenderWindow * render_window, const Ogre::ColourValue * color);

  static
  void
  setDirectionalLightDirection(RenderWindow * render_window, const Ogre::Vector3 & vec);

  static
  Ogre::Light *
  getDirectionalLight(RenderWindow * render_window);

  static
  Ogre::SceneManager *
  getSceneManager(RenderWindow * render_window);

  static
  void
  setSceneManager(RenderWindow * render_window, Ogre::SceneManager * scene_manager);

  static
  void
  addListener(RenderWindow * render_window, Ogre::RenderTargetListener * listener);

  static
  void
  setVisibilityMask(RenderWindow * render_window, uint32_t mask);

  static
  void
  removeListener(RenderWindow * render_window, Ogre::RenderTargetListener * listener);
};

}  // namespace rviz_rendering

#endif  // RVIZ_RENDERING__RENDER_WINDOW_HPP_
