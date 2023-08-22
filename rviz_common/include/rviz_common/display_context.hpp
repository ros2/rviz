/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

#ifndef RVIZ_COMMON__DISPLAY_CONTEXT_HPP_
#define RVIZ_COMMON__DISPLAY_CONTEXT_HPP_

#include <cstdint>
#include <memory>

#include <QObject>  // NOLINT: cpplint is unable to handle the include order here
#include <QString>  // NOLINT: cpplint is unable to handle the include order here

#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"
#include "rviz_common/visibility_control.hpp"
#include "frame_manager_iface.hpp"

class QKeyEvent;

namespace Ogre
{
class SceneManager;
}  // namespace Ogre

namespace rclcpp
{
class Clock;
}  // namespace rclcpp

// namespace tf
// {
// class TransformListener;
// }

namespace rviz_common
{

namespace interaction
{

class SelectionManagerIface;
class HandlerManagerIface;
class ViewPickerIface;

}  // namespace interaction

namespace transformation
{

class TransformationManager;

}  // namespace transformation

class BitAllocator;
class DisplayFactory;
class DisplayGroup;
class FrameManager;
class RenderPanel;
class ToolManager;
class ViewportMouseEvent;
class ViewManager;
class WindowManagerInterface;

/// Pure-virtual base class for objects which give Display subclasses context in which to work.
/**
 * This interface class mainly exists to enable more isolated unit
 * tests by enabling small mock objects to take the place of the large
 * VisualizationManager implementation class.
 * It also serves to define a narrower, and more maintainable API for use in
 * the Display plugins.
 */
class RVIZ_COMMON_PUBLIC DisplayContext : public QObject
{
  Q_OBJECT

public:
  // TODO(wjwwood): I disabled many of the convenience functions so that each
  //                use of them will cause a compiler error, at which point
  //                we can evaluate if that function needs to be reenabled or
  //                the calling code needs to be changed on a case by case basis.
  //                Once we've done that, the TODO is to remove the still
  //                disabled functions from this interface to keep it small
  //                and to keep it with as few dependencies as possible.

  // TODO(wjwwood): refactor this to return something from rviz_rendering without Ogre
  /// Returns the Ogre::SceneManager used for the main RenderPanel.
  virtual Ogre::SceneManager * getSceneManager() const = 0;

  /// Return the window manager, if any.
  virtual
  WindowManagerInterface *
  getWindowManager() const = 0;

  /// Return a pointer to the SelectionManager.
  virtual
  std::shared_ptr<rviz_common::interaction::SelectionManagerIface>
  getSelectionManager() const = 0;

  /// Return a pointer to the HandlerManager.
  virtual
  std::shared_ptr<rviz_common::interaction::HandlerManagerIface>
  getHandlerManager() const = 0;

  /// Return a pointer to the ViewPicker.
  virtual
  std::shared_ptr<rviz_common::interaction::ViewPickerIface>
  getViewPicker() const = 0;

  /// Return the FrameManager instance.
  virtual
  FrameManagerIface *
  getFrameManager() const = 0;

  /// Return the fixed frame name.
  virtual
  QString
  getFixedFrame() const = 0;

  // TODO(wjwwood): replace uint64_t with size_t unless there is a reason for it
  /// Return the current value of the frame count.
  /**
   * The frame count is just a number which increments each time a
   * frame is rendered.  This lets clients check if a new frame has
   * been rendered since the last time they did something.
   */
  virtual
  uint64_t
  getFrameCount() const = 0;

  /// Return a factory for creating Display subclasses based on a class id string.
  virtual
  DisplayFactory *
  getDisplayFactory() const = 0;

  /// Return a weak pointer to the ros node (abstraction) used by rviz
  virtual
  ros_integration::RosNodeAbstractionIface::WeakPtr
  getRosNodeAbstraction() const = 0;

  /// Handle a single key event for a given RenderPanel.
  virtual
  void
  handleChar(QKeyEvent * event, RenderPanel * panel) = 0;

  /// Handle a mouse event.
  virtual
  void
  handleMouseEvent(const ViewportMouseEvent & event) = 0;

  /// Return the ToolManager.
  virtual
  ToolManager *
  getToolManager() const = 0;

  /// Return the ViewManager.
  virtual
  ViewManager *
  getViewManager() const = 0;

  virtual
  transformation::TransformationManager *
  getTransformationManager() = 0;

  /// Return the root DisplayGroup.
  virtual
  DisplayGroup *
  getRootDisplayGroup() const = 0;

  /// Get the default visibility bit.
  virtual
  uint32_t
  getDefaultVisibilityBit() const = 0;

  /// Get the visibility bits.
  virtual
  BitAllocator *
  visibilityBits() = 0;

  /// Set the message displayed in the status bar.
  virtual
  void
  setStatus(const QString & message) = 0;

  virtual
  QString
  getHelpPath() const = 0;

  virtual
  std::shared_ptr<rclcpp::Clock>
  getClock() = 0;

  virtual
  void
  lockRender() = 0;

  virtual
  void
  unlockRender() = 0;

public Q_SLOTS:
  /// Queue a render.
  /**
   * Multiple calls before a render happens will only cause a single render.
   *
   * \note This function can be called from any thread.
   */
  virtual
  void
  queueRender() = 0;
};

}  // namespace rviz_common

#endif  // RVIZ_COMMON__DISPLAY_CONTEXT_HPP_
