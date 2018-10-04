/*
 * Copyright (c) 2009, Willow Garage, Inc.
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

#ifndef RVIZ_COMMON__WINDOW_MANAGER_INTERFACE_HPP_
#define RVIZ_COMMON__WINDOW_MANAGER_INTERFACE_HPP_

#include <Qt>

class QWidget;
class QString;

namespace rviz_common
{

class PanelDockWidget;

/// Pure virtual class which represents the interface for adding panels to the main rviz frame.
/**
 * This class is useful as a way to moc a small part of the VisualizationFrame
 * class while testing without having to moc the whole thing.
 */
class WindowManagerInterface
{
public:
  virtual ~WindowManagerInterface() = default;

  /// Return the parent QWidget.
  virtual
  QWidget *
  getParentWindow() = 0;

  /// Add a pane to the visualizer.
  /**
   * To remove a pane, delete it.
   * Other operations can also be done directly to the PanelDockWidget:
   * show(), hide(), close(), etc.
   */
  virtual
  PanelDockWidget *
  addPane(
    const QString & name,
    QWidget * pane,
    Qt::DockWidgetArea area = Qt::LeftDockWidgetArea,
    bool floating = true) = 0;

  /// Set the message displayed in the status bar.
  virtual
  void
  setStatus(const QString & message) = 0;
};

}  // namespace rviz_common

#endif  // RVIZ_COMMON__WINDOW_MANAGER_INTERFACE_HPP_
