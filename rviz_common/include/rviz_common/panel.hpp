/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * Copyright (c) 2017, Open Source Robotics Foundation, Inc.
 * Copyright (c) 2018, Bosch Software Innovations GmbH.
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

#ifndef RVIZ_COMMON__PANEL_HPP_
#define RVIZ_COMMON__PANEL_HPP_

#include <QWidget>

#include "rviz_common/config.hpp"

namespace rviz_common
{

class DisplayContext;

class RVIZ_COMMON_PUBLIC Panel : public QWidget
{
  Q_OBJECT

public:
  explicit Panel(QWidget * parent = nullptr);
  ~Panel() override;

  /// Initialize the panel with a DisplayContext.
  /**
   * Called by VisualizationFrame during setup.
   */
  void initialize(DisplayContext * context);

  /// Override-able function to do initialization.
  /**
   * Override to do initialization which depends on the DisplayContext being available.
   * The default implementation does nothing.
   */
  virtual void onInitialize();

  /// Return the name.
  virtual QString getName() const;

  /// Set the name.
  virtual void setName(const QString & name);

  /// Return a description of this Panel.
  virtual QString getDescription() const;

  /// Set a description of this Panel.
  /**
   * Called by the factory which creates it.
   */
  virtual void setDescription(const QString & description);

  /// Return the class identifier which was used to create this instance.
  /**
   * This version just returns whatever was set with setClassId().
   */
  virtual QString getClassId() const;

  /// Set the class identifier used to create this instance.
  /**
   * Typically this will be set by the factory object which created it.
   */
  virtual void setClassId(const QString & class_id);

  /// Override to load configuration data.
  /**
   * This version loads the name of the panel.
   */
  virtual void load(const Config & config);

  /// Override to save configuration data.
  /**
   * This version saves the name and class ID of the panel.
   */
  virtual void save(Config config) const;

Q_SIGNALS:
  /// Subclasses must emit this whenever a configuration change happens.
  /**
   * This is used to let the system know that changes have been made
   * since the last time the config was saved.
   */
  void configChanged();

protected:
  DisplayContext * getDisplayContext() const;

private:
  DisplayContext * context_;
  QString class_id_;
  QString name_;
  QString description_;
};

}  // namespace rviz_common

#endif  // RVIZ_COMMON__PANEL_HPP_
