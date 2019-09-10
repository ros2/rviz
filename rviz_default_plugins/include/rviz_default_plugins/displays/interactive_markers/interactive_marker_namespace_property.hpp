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
#ifndef /* NOLINT */ \
  RVIZ_DEFAULT_PLUGINS__DISPLAYS__INTERACTIVE_MARKERS__INTERACTIVE_MARKER_NAMESPACE_PROPERTY_HPP_
#define /* NOLINT */ \
  RVIZ_DEFAULT_PLUGINS__DISPLAYS__INTERACTIVE_MARKERS__INTERACTIVE_MARKER_NAMESPACE_PROPERTY_HPP_

#include <string>

#include "rviz_common/properties/editable_enum_property.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"

#include "rviz_default_plugins/visibility_control.hpp"

namespace rviz_default_plugins
{
namespace displays
{

class RVIZ_DEFAULT_PLUGINS_PUBLIC InteractiveMarkerNamespaceProperty
  : public rviz_common::properties::EditableEnumProperty
{
  Q_OBJECT

public:
  explicit InteractiveMarkerNamespaceProperty(
    const QString & name = QString(),
    const QString & default_value = QString(),
    const QString & description = QString(),
    rviz_common::properties::Property * parent = nullptr,
    const char * changed_slot = nullptr,
    QObject * receiver = nullptr);

  void initialize(rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node);

  QString getNamespace() const
  {return getValue().toString();}

  std::string getNamespaceStd() const
  {return getValue().toString().toStdString();}

  bool isEmpty() const
  {return getNamespaceStd().empty();}

protected Q_SLOTS:
  virtual void fillNamespaceList();

private:
  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node_;
};

}  // namespace displays
}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__INTERACTIVE_MARKERS__INTERACTIVE_MARKER_NAMESPACE_PROPERTY_HPP_
