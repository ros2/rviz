/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

#include <map>
#include <string>
#include <vector>

#include <QApplication>  // NOLINT: cpplint can't handle Qt imports

#include "rviz_common/properties/ros_topic_property.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"

namespace rviz_common
{
namespace properties
{

RosTopicProperty::RosTopicProperty(
  const QString & name,
  const QString & default_value,
  const QString & message_type,
  const QString & description,
  Property * parent,
  const char * changed_slot,
  QObject * receiver)
: EditableEnumProperty(name, default_value, description, parent, changed_slot, receiver),
  rviz_ros_node_(),
  message_type_(message_type)
{
  connect(
    this, SIGNAL(requestOptions(EditableEnumProperty*)),
    this, SLOT(fillTopicList()));
}

void RosTopicProperty::initialize(ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node)
{
  rviz_ros_node_ = rviz_ros_node;
}

void RosTopicProperty::setMessageType(const QString & message_type)
{
  message_type_ = message_type;
}

void RosTopicProperty::fillTopicList()
{
  QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));
  clearOptions();

  std::string std_message_type = message_type_.toStdString();
  std::map<std::string, std::vector<std::string>> published_topics =
    rviz_ros_node_.lock()->get_topic_names_and_types();

  for (const auto & topic : published_topics) {
    // Only add topics whose type matches.
    for (const auto & type : topic.second) {
      // TODO(Martin-Idel-SI): revisit after message_traits become available.
      // We only want to show the types of the topic we subscribe to, however, currently we can't
      // get the type, so std_message_type will always be empty --> show all topics instead
      if (std_message_type.empty() || type == std_message_type) {
        addOptionStd(topic.first);
      }
    }
  }
  sortOptions();
  QApplication::restoreOverrideCursor();
}

}  // end namespace properties

}  // end namespace rviz_common
