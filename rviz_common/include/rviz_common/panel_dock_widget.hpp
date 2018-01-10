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

#ifndef RVIZ_COMMON__PANEL_DOCK_WIDGET_HPP_
#define RVIZ_COMMON__PANEL_DOCK_WIDGET_HPP_

#include <QDockWidget>
#include <QLabel>

#include "rviz_common/config.hpp"
#include "rviz_common/visibility_control.hpp"

namespace rviz_common
{

/// Dock widget class for docking widgets into VisualizationFrame.
/**
 * Use setContentWidget() instead of QDockWidget::setWidget() if you
 * want the PanelDockWidget to be destroyed when the content widget is
 * destroyed.
 */
class RVIZ_COMMON_PUBLIC PanelDockWidget : public QDockWidget
{
  Q_OBJECT

public:
  explicit PanelDockWidget(const QString & name);

  /// Set the widget which is the main content of the panel.
  void setContentWidget(QWidget * child);

  /// Collapse the panel.
  void setCollapsed(bool collapsed);

  /// Set the icon for the panel.
  void setIcon(QIcon icon);

  /// Save to a given Config object.
  virtual void save(Config config);

  /// Load to a given Config object.
  virtual void load(Config config);

  /// Override setVisible to respect the visibility override.
  void setVisible(bool visible) override;

protected:
  /// Called when the user closes the panel or ancestor.
  void closeEvent(QCloseEvent * event) override;

public Q_SLOTS:
  /// Called when the set window title signal is emitted.
  void setWindowTitle(QString title);

  /// Override the visibility of the widget.
  virtual void overrideVisibility(bool hide);

private Q_SLOTS:
  /// Called when a child widget is destroyed.
  void onChildDestroyed(QObject *);

Q_SIGNALS:
  /// Called when the panel is closed.
  void closed();

private:
  // set to true if this panel was collapsed
  bool collapsed_;
  bool requested_visibility_;
  bool forced_hidden_;
  QLabel * icon_label_;
  QLabel * title_label_;
};

}  // namespace rviz_common

#endif  // RVIZ_COMMON__PANEL_DOCK_WIDGET_HPP_
