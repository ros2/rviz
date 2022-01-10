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

#include "panel_factory.hpp"

#include <string>

#include "displays_panel.hpp"
#include "help_panel.hpp"
#include "selection_panel.hpp"
#include "time_panel.hpp"
#include "tool_properties_panel.hpp"
#include "transformation_panel.hpp"
#include "views_panel.hpp"
#include "rviz_common/visualization_manager.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"

namespace rviz_common
{

static Panel * newHelpPanel() {return new HelpPanel();}
static Panel * newSelectionPanel() {return new SelectionPanel();}
static Panel * newToolPropertiesPanel() {return new ToolPropertiesPanel();}
static Panel * newTransformationPanel() {return new TransformationPanel();}
static Panel * newViewsPanel() {return new ViewsPanel();}

PanelFactory::PanelFactory(
  ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node,
  VisualizationManager * manager)
: PluginlibFactory<Panel>("rviz_common", "rviz_common::Panel")
{
  addBuiltInClass(
    "rviz_common", "Displays",
    "Show and edit the list of Displays",
    [rviz_ros_node, manager]() -> Panel * {
      return new DisplaysPanel(rviz_ros_node, manager, nullptr);
    });
  addBuiltInClass(
    "rviz_common", "Help",
    "Show the key and mouse bindings", &newHelpPanel);
  addBuiltInClass(
    "rviz_common", "Selection",
    "Show properties of selected objects", &newSelectionPanel);
  addBuiltInClass(
    "rviz_common", "Time",
    "Show the current time",
    [manager]() -> Panel * {
      return new TimePanel(manager, nullptr);
    });
  addBuiltInClass(
    "rviz_common", "Tool Properties",
    "Show and edit properties of tools", &newToolPropertiesPanel);
  addBuiltInClass(
    "rviz_common", "Transformation",
    "Choose the transformation plugin", &newTransformationPanel);
  addBuiltInClass(
    "rviz_common", "Views",
    "Show and edit viewpoints", &newViewsPanel);
}

}  // end namespace rviz_common
