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

#include "rviz_common/tool_manager.hpp"

#include <cassert>
#include <vector>

#include <QKeyEvent>  // NOLINT: cpplint is unable to handle the include order here
#include <QKeySequence>  // NOLINT: cpplint is unable to handle the include order here
#include <QRegExp>  // NOLINT: cpplint is unable to handle the include order here

#include "rviz_common/logging.hpp"

#include "./failed_tool.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/properties/property.hpp"
#include "rviz_common/properties/property_tree_model.hpp"

namespace rviz_common
{

using rviz_common::properties::Property;
using rviz_common::properties::PropertyTreeModel;

QString addSpaceToCamelCase(QString input)
{
  QRegExp re = QRegExp("([A-Z])([a-z]*)");
  input.replace(re, " \\1\\2");
  return input.trimmed();
}

ToolManager::ToolManager(DisplayContext * context)
: factory_(new PluginlibFactory<Tool>("rviz_common", "rviz_common::Tool")),
  property_tree_model_(new PropertyTreeModel(new Property())),
  context_(context),
  current_tool_(nullptr),
  default_tool_(nullptr)
{
  connect(property_tree_model_.get(), SIGNAL(configChanged()), this, SIGNAL(configChanged()));
}

ToolManager::~ToolManager()
{
  removeAll();
}

void ToolManager::initialize()
{
  // get a list of available tool plugin class ids
  auto plugins = factory_->getDeclaredPlugins();
  // define a list of preferred tool names (they will be listed first in the toolbar)
  std::vector<const char *> preferred_tool_names = {
    "rviz_default_plugins/MoveCamera",
    "rviz_default_plugins/Interact",
    "rviz_default_plugins/Select",
    "rviz_default_plugins/SetInitialPose",
    "rviz_default_plugins/SetGoal",
  };
  // attempt to load each preferred tool in order
  for (const auto & preferred_tool_name : preferred_tool_names) {
    for (const auto & plugin : plugins) {
      if (plugin.name.toStdString() == preferred_tool_name) {
        addTool(plugin);
      }
    }
  }
  // other tools will need to be loaded manually by the user through the UI
}

void ToolManager::removeAll()
{
  for (int i = tools_.size() - 1; i >= 0; i--) {
    removeTool(i);
  }
}

void ToolManager::load(const Config & config)
{
  removeAll();

  int num_tools = config.listLength();
  for (int i = 0; i < num_tools; i++) {
    Config tool_config = config.listChildAt(i);

    QString class_id;
    if (tool_config.mapGetString("Class", &class_id)) {
      Tool * tool = addTool(factory_->getPluginInfo(class_id));
      tool->load(tool_config);
    }
  }
}

void ToolManager::save(Config config) const
{
  for (int i = 0; i < tools_.size(); i++) {
    tools_[i]->save(config.listAppendNew());
  }
}

rviz_common::properties::PropertyTreeModel * ToolManager::getPropertyModel() const
{
  return property_tree_model_.get();
}

bool ToolManager::toKey(QString const & str, uint & key)
{
  QKeySequence seq(str);

  // We should only working with a single key here
  if (seq.count() == 1) {
    key = seq[0];
    return true;
  } else {
    return false;
  }
}

void ToolManager::handleChar(QKeyEvent * event, RenderPanel * panel)
{
  // if the incoming key is ESC fallback to the default tool
  if (event->key() == Qt::Key_Escape) {
    setCurrentTool(getDefaultTool());
    return;
  }

  // check if the incoming key triggers the activation of another tool
  Tool * tool = nullptr;
  if (shortkey_to_tool_map_.find(event->key()) != shortkey_to_tool_map_.end()) {
    tool = shortkey_to_tool_map_[event->key()];
  }

  if (tool) {
    // if there is a incoming tool check if it matches the current tool
    if (current_tool_ == tool) {
      // if yes, deactivate the current tool and fallback to default
      setCurrentTool(getDefaultTool());
    } else {
      // if no, check if the current tool accesses all key events
      if (current_tool_->accessAllKeys()) {
        // if yes, pass the key
        current_tool_->processKeyEvent(event, panel);
      } else {
        // if no, switch the tool
        setCurrentTool(tool);
      }
    }
  } else {
    // if the incoming key triggers no other tool,
    // just hand down the key event
    current_tool_->processKeyEvent(event, panel);
  }
}

void ToolManager::setCurrentTool(Tool * tool)
{
  if (current_tool_) {
    current_tool_->deactivate();
  }

  current_tool_ = tool;

  if (current_tool_) {
    current_tool_->activate();
  }

  Q_EMIT toolChanged(current_tool_);
}

void ToolManager::setDefaultTool(Tool * tool)
{
  default_tool_ = tool;
}

Tool * ToolManager::getTool(int index)
{
  assert(index >= 0);
  assert(index < static_cast<int>(tools_.size()));

  return tools_[index];
}

void ToolManager::updatePropertyVisibility(rviz_common::properties::Property * container)
{
  if (container->numChildren() > 0) {
    if (!property_tree_model_->getRoot()->contains(container)) {
      property_tree_model_->getRoot()->addChild(container);
      container->expand();
    }
  } else {
    property_tree_model_->getRoot()->takeChild(container);
  }
}

void ToolManager::closeTool()
{
  setCurrentTool(getDefaultTool());
}

Tool * ToolManager::addTool(const QString & class_id)
{
  return addTool(factory_->getPluginInfo(class_id));
}

Tool * ToolManager::addTool(const PluginInfo & tool_plugin)
{
  QString error;
  bool failed = false;
  Tool * tool = factory_->make(tool_plugin.id, &error);
  if (!tool) {
    tool = new FailedTool(tool_plugin.id, error);
    failed = true;
  }

  tools_.append(tool);
  tool->setName(addSpaceToCamelCase(tool_plugin.name));
  tool->setIcon(tool_plugin.icon);
  tool->initialize(context_);

  if (tool->getShortcutKey() != '\0') {
    uint key;
    QString str = QString(tool->getShortcutKey());

    if (toKey(str, key)) {
      shortkey_to_tool_map_[key] = tool;
    }
  }

  rviz_common::properties::Property * container = tool->getPropertyContainer();
  connect(
    container,
    SIGNAL(childListChanged(rviz_common::properties::Property*)),
    this,
    SLOT(updatePropertyVisibility(rviz_common::properties::Property*)));
  updatePropertyVisibility(container);

  Q_EMIT toolAdded(tool);

  // If the default tool is unset and this tool loaded correctly, set
  // it as the default and current.
  if (default_tool_ == nullptr && !failed) {
    setDefaultTool(tool);
    setCurrentTool(tool);
  }

  QObject::connect(tool, SIGNAL(close()), this, SLOT(closeTool()));

  Q_EMIT configChanged();

  return tool;
}

void ToolManager::removeTool(int index)
{
  Tool * tool = tools_.takeAt(index);
  Tool * fallback = nullptr;
  if (tools_.size() > 0) {
    fallback = tools_[0];
  }
  if (tool == current_tool_) {
    setCurrentTool(fallback);
  }
  if (tool == default_tool_) {
    setDefaultTool(fallback);
  }
  Q_EMIT toolRemoved(tool);

  uint key;
  if (toKey(QString(tool->getShortcutKey()), key)) {
    shortkey_to_tool_map_.erase(key);
  }
  delete tool;
  Q_EMIT configChanged();
}

void ToolManager::refreshTool(Tool * tool)
{
  Q_EMIT toolRefreshed(tool);
}

QStringList ToolManager::getToolClasses()
{
  QStringList class_names;
  for (int i = 0; i < tools_.size(); i++) {
    class_names.append(tools_[i]->getClassId());
  }
  return class_names;
}

Tool * ToolManager::getCurrentTool()
{
  return current_tool_;
}

int ToolManager::numTools()
{
  return tools_.size();
}

Tool * ToolManager::getDefaultTool()
{
  return default_tool_;
}

PluginlibFactory<Tool> * ToolManager::getFactory()
{
  return factory_.get();
}

}  // namespace rviz_common
