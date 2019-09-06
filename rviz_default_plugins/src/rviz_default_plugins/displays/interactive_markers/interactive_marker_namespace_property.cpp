/*
 * Copyright (c) 2019, Open Source Robotics Foundation, Inc.
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
 *     * Neither the name of the copyright holder nor the names of its
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

#include <QApplication>

#include <map>
#include <string>
#include <vector>

#include "rviz_common/logging.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"

#include \
  "rviz_default_plugins/displays/interactive_markers/interactive_marker_namespace_property.hpp"

namespace rviz_default_plugins
{
namespace displays
{

InteractiveMarkerNamespaceProperty::InteractiveMarkerNamespaceProperty(
  const QString & name,
  const QString & default_value,
  const QString & description,
  rviz_common::properties::Property * parent,
  const char * changed_slot,
  QObject * receiver)
: rviz_common::properties::EditableEnumProperty(
    name, default_value, description, parent, changed_slot, receiver),
  rviz_ros_node_()
{
  connect(this, SIGNAL(requestOptions(EditableEnumProperty*)), this, SLOT(fillNamespaceList()));
}

void InteractiveMarkerNamespaceProperty::initialize(
  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node)
{
  rviz_ros_node_ = rviz_ros_node;
}

void InteractiveMarkerNamespaceProperty::fillNamespaceList()
{
  QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));
  clearOptions();

  auto rviz_ros_node_ptr = rviz_ros_node_.lock();
  if (!rviz_ros_node_ptr) {
    RVIZ_COMMON_LOG_ERROR(
      "RViz ROS node is null in InteractiveMarkerNamespaceProperty. Was initialize() called?");
    return;
  }

  // TODO(jacobperron): Only get service server names when/if an rclcpp API becomes available
  std::map<std::string, std::vector<std::string>> services =
    rviz_ros_node_ptr->get_raw_node()->get_service_names_and_types();

  // Add namespaces of service names that have the expected type:
  // visualization_msgs/srv/GetInteractiveMarkers
  for (const auto & service : services) {
    for (const auto & type : service.second) {
      if ("visualization_msgs/srv/GetInteractiveMarkers" == type) {
        const std::string service_fqn = service.first;
        const std::string service_namespace = service_fqn.substr(0, service_fqn.rfind("/"));
        addOptionStd(service_namespace);
      }
    }
  }
  sortOptions();
  QApplication::restoreOverrideCursor();
}

}  // namespace displays
}  // namespace rviz_default_plugins
