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

#ifndef RVIZ_COMMON__TOOL_MANAGER_HPP_
#define RVIZ_COMMON__TOOL_MANAGER_HPP_

#include <map>
#include <memory>

#include <QList>  // NOLINT: cpplint is unable to handle the include order here
#include <QObject>  // NOLINT: cpplint is unable to handle the include order here
#include <QStringList>  // NOLINT: cpplint is unable to handle the include order here

#include "rviz_common/factory/pluginlib_factory.hpp"
#include "rviz_common/tool.hpp"

class QKeyEvent;

namespace rviz_common
{

namespace properties
{

class PropertyTreeModel;

}  // namespace properties

class ToolManager : public QObject
{
  Q_OBJECT

public:
  explicit ToolManager(DisplayContext * context);
  virtual ~ToolManager();

  /// Initialization for after the DisplayContext is created.
  /**
   * Loads standard RViz tools.
   */
  void initialize();

  /// Load settings for tool manager from Config object.
  void load(const Config & config);

  /// Save settings of tool manager to Config object.
  void save(Config config) const;

  /// Get the property tree model.
  rviz_common::properties::PropertyTreeModel * getPropertyModel() const;

  /// Create a tool by class id, add it to the list, and return it.
  Tool * addTool(const QString & class_id);

  /// Create a tool by plugin info, add it to the list, and return it.
  Tool * addTool(const PluginInfo & tool_plugin);

  /// Return the tool currently in use.
  /**
   * \see setCurrentTool()
   */
  Tool * getCurrentTool();

  /// Return the tool at a given index in the Tool list.
  /**
   * If index is less than 0 or greater than the number of tools, this
   * will fail an assertion.
   */
  Tool * getTool(int index);

  /// Get the number of tools.
  int numTools();

  /// Remove tool by index.
  void removeTool(int index);

  /// Removal all the tools.
  void removeAll();

  /// Triggers redrawing the tool's icon/text in the toolbar.
  void refreshTool(Tool * tool);

  /// Set the current tool.
  /**
   * The current tool is given all mouse and keyboard events which
   * VisualizationManager receives via handleMouseEvent() and handleChar().
   * \see getCurrentTool()
   */
  void setCurrentTool(Tool * tool);

  /// Set the default tool.
  /**
   * The default tool is selected directly by pressing the Escape key.
   * The default tool is indirectly selected when a Tool returns
   * Finished in the bit field result of Tool::processMouseEvent().
   * This is how control moves from the InitialPoseTool back to
   * MoveCamera when InitialPoseTool receives a left mouse button
   * release event.
   * \see getDefaultTool()
   */
  void setDefaultTool(Tool * tool);

  /// Get the default tool.
  /**
   * \see setDefaultTool()
   */
  Tool * getDefaultTool();

  /// Get the names of the tool classes.
  QStringList getToolClasses();

  /// Function to handle a key event.
  void handleChar(QKeyEvent * event, RenderPanel * panel);

  PluginlibFactory<Tool> * getFactory();

Q_SIGNALS:
  /// Emitted when anything changes which will change the saved config file contents.
  void configChanged();

  /// Emitted by addTool() after the tool is added to the list of tools.
  void toolAdded(Tool *);

  /// Emitted by setCurrentTool() after the newly chosen tool is activated.
  void toolChanged(Tool *);

  /// Emitted when a tool is removed.
  void toolRemoved(Tool *);

  /// Emitted by refreshTool() to gedraw the tool's icon in the toolbar.
  void toolRefreshed(Tool *);

private Q_SLOTS:
  /// If property has children, add it to the tool property tree, else remove it.
  void updatePropertyVisibility(rviz_common::properties::Property * property);

  /// Deactivate the current tool and sets the default tool.
  void closeTool();

private:
  /// Convert a key string to the unsiged integer which represents it.
  /**
   * Returns false if the conversion fails.
   */
  bool toKey(QString const & str, uint & key_out);

  std::unique_ptr<PluginlibFactory<Tool>> factory_;
  std::unique_ptr<rviz_common::properties::PropertyTreeModel> property_tree_model_;
  QList<Tool *> tools_;
  DisplayContext * context_;
  Tool * current_tool_;
  Tool * default_tool_;
  std::map<int, Tool *> shortkey_to_tool_map_;
};

}  // namespace rviz_common

#endif  // RVIZ_COMMON__TOOL_MANAGER_HPP_
