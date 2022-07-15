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

#ifndef RVIZ_COMMON__RENDER_PANEL_HPP_
#define RVIZ_COMMON__RENDER_PANEL_HPP_

#include <map>
#include <memory>
#include <mutex>
#include <vector>

#include <OgreVector.h>

#include <QWidget>  // NOLINT: cpplint is unable to handle the include order here

#include "rviz_common/visibility_control.hpp"

class QGridLayout;
class QMenu;

namespace rviz_rendering
{

class RenderWindow;

}  // namespace rviz_rendering

namespace rviz_common
{

class Display;
class DisplayContext;
class ViewController;

/**
 * A widget which shows a 3D scene in RViz.
 *
 * RenderPanel displays a scene and forwards mouse and key events to
 * the DisplayContext (which further forwards them to the active Tool, etc.)
 */
class RVIZ_COMMON_PUBLIC RenderPanel : public QWidget
{
  Q_OBJECT

public:
  explicit RenderPanel(QWidget * parent = 0);
  virtual ~RenderPanel();

  /// Initialize the render panel.
  /**
    * This sets up the Camera for this widget.
    */
  void initialize(DisplayContext * manager, bool use_main_scene = false);
  // void initialize(Ogre::SceneManager * scene_manager, DisplayContext * manager);

  DisplayContext * getManager();

  ViewController * getViewController();

  /// Set the ViewController which should control the camera position for this view.
  void setViewController(ViewController * controller);

  /// Get the RenderWindow.
  rviz_rendering::RenderWindow * getRenderWindow();

  /// Overrides the default implementation.
  /**
   * This override is here for convenience. Returns a symbolic 320x240px size.
   * \return A size of 320x240 (just a symbolic 4:3 size).
   */
  QSize
  sizeHint() const override;

  // This method has to be overridden in order to avoid an Ogre assertion to fail on macOS
  // (The assertion can be found in OgreOSXCocoaWindow.mm, line 410)
  void resizeEvent(QResizeEvent * event) override;

  static const Ogre::Vector3 default_camera_pose_;

  /// Show the given menu as a context menu, positioned based on the current mouse position.
  /**
   * This can be called from any thread.
   */
  void showContextMenu(std::shared_ptr<QMenu> menu);

  /// Return true if the context menu for this panel is visible.
  bool contextMenuVisible();

  // TODO(wjwwood): this should be moved into rviz_rendering::RenderWindowImpl, I think
  // virtual void sceneManagerDestroyed(Ogre::SceneManager * source);

protected:
  /// Called when any mouse event happens inside the render window.
  void onRenderWindowMouseEvents(QMouseEvent * event);

  /* Start QWidget overrides. */

  // TODO(wjwwood): I think most of these can be handled in the rviz_rendering::RenderWindowImpl
  /// Called when a context menu event occurs in the render window.
  void contextMenuEvent(QContextMenuEvent * event) override;

  /// Called when the mouse moved in the render window.
  void mouseMoveEvent(QMouseEvent * event) override;

  /// Called when a mouse button is pressed down in the render window.
  void mousePressEvent(QMouseEvent * event) override;

  /// Called when a mouse button is released in the render window.
  void mouseReleaseEvent(QMouseEvent * event) override;

  /// Called when a mouse button is double-clicked in the render window.
  void mouseDoubleClickEvent(QMouseEvent * event) override;

  /// Called when a mouse leaves the render window.
  void leaveEvent(QEvent * event) override;

  /// Called when there is a mouse-wheel event in the render window.
  void wheelEvent(QWheelEvent * event) override;

  /// Called when there is a key pressed in the render window.
  void keyPressEvent(QKeyEvent * event) override;

  /* End QWidget overrides. */

  /// X position of the last mouse event.
  int mouse_x_;
  /// Y position of the last mouse event.
  int mouse_y_;

  DisplayContext * context_;
  // Ogre::SceneManager * scene_manager_;

  ViewController * view_controller_;

  std::shared_ptr<QMenu> context_menu_;
  std::mutex context_menu_mutex_;

  bool context_menu_visible_;

  // Pointer to the Display which is using this render panel, or nullptr
  // if this does not belong to a Display.
  Display * display_;

  rviz_rendering::RenderWindow * render_window_;
  QWidget * render_window_container_widget_;
  QGridLayout * layout_;

private Q_SLOTS:
  void sendMouseMoveEvent();
  void onContextMenuHide();

private:
  QTimer * fake_mouse_move_event_timer_;

  // TODO(wjwwood): I think this can be stored in rviz_rendering::RenderWindowImpl
  /// A default camera created in initialize().
  // Ogre::Camera * default_camera_;
};

}  // namespace rviz_common

#endif  // RVIZ_COMMON__RENDER_PANEL_HPP_
