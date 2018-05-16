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

#ifndef RVIZ_COMMON__DISPLAYS_PANEL_HPP_
#define RVIZ_COMMON__DISPLAYS_PANEL_HPP_

#include <map>
#include <set>
#include <string>
#include <vector>

#include "rviz_common/config.hpp"
#include "rviz_common/panel.hpp"

class QPushButton;

namespace rviz_common
{

namespace properties
{

class PropertyTreeWidget;
class PropertyTreeWithHelp;

}  // namespace properties

class VisualizationManager;
class Display;

class DisplaysPanel : public Panel
{
  Q_OBJECT

public:
  explicit DisplaysPanel(const std::string & node_name, QWidget * parent = 0);
  virtual ~DisplaysPanel();

  void onInitialize() override;

  void save(Config config) const override;

  void load(const Config & config) override;

protected Q_SLOTS:
  /// Called when the "Add" button is pressed
  void onNewDisplay();
  /// Called when the "copy" button is pressed
  void onDuplicateDisplay();
  /// Called when the "Remove" button is pressed
  void onDeleteDisplay();
  /// Called when the "Rename" button is pressed
  void onRenameDisplay();

  void onSelectionChanged();

protected:
  properties::PropertyTreeWidget * property_grid_;

  QPushButton * duplicate_button_;
  QPushButton * remove_button_;
  QPushButton * rename_button_;
  properties::PropertyTreeWithHelp * tree_with_help_;
  const std::string node_name_;
};

}  // namespace rviz_common

#endif  // RVIZ_COMMON__DISPLAYS_PANEL_HPP_
