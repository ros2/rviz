// Copyright (c) 2025, Open Source Robotics Foundation, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <QApplication>  // NOLINT: cpplint can't handle Qt imports
#include <QObject>  // NOLINT: cpplint can't handle Qt imports
#include <QRegExp>  // NOLINT: cpplint can't handle Qt imports
#include <QString>  // NOLINT: cpplint can't handle Qt imports
#include <QStringList>  // NOLINT: cpplint can't handle Qt imports

#include "rviz_common/properties/ros_action_property.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"

namespace rviz_common
{
namespace properties
{

RosActionProperty::RosActionProperty(
  const QString & name,
  const QString & default_value,
  const QString & action_type,
  const QString & description,
  Property * parent,
  const char * changed_slot,
  QObject * receiver)
: EditableEnumProperty(name, default_value, description, parent, changed_slot, receiver),
  rviz_ros_node_(),
  action_type_(action_type)
{
  connect(
    this, SIGNAL(requestOptions(EditableEnumProperty*)),
    this, SLOT(fillActionList()));
}

void RosActionProperty::initialize(ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node)
{
  rviz_ros_node_ = rviz_ros_node;
}

void RosActionProperty::setActionType(const QString & action_type)
{
  action_type_ = action_type;
}

QString RosActionProperty::getActionType() const
{
  return action_type_;
}

QString RosActionProperty::getAction() const
{
  return getValue().toString();
}

std::string RosActionProperty::getActionStd() const
{
  return getValue().toString().toStdString();
}

bool RosActionProperty::isEmpty() const
{
  return getActionStd().empty();
}

void RosActionProperty::fillActionList()
{
  QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));
  clearOptions();

  std::string std_action_type = action_type_.toStdString() + "_FeedbackMessage";
  std::string suffix = "/_action/feedback";
  std::map<std::string, std::vector<std::string>> published_topics =
    rviz_ros_node_.lock()->get_topic_names_and_types();

  for (const auto & topic : published_topics) {
    // Only add topics whose type matches.
    for (const auto & type : topic.second) {
      if (type == std_action_type) {
        auto action = topic.first.substr(0, topic.first.size() - suffix.size());
        addOptionStd(action);
      }
    }
  }
  sortOptions();
  QApplication::restoreOverrideCursor();
}

RosFilteredActionProperty::RosFilteredActionProperty(
  const QString & name,
  const QString & default_value,
  const QString & action_type,
  const QString & description,
  const QRegExp & filter,
  Property * parent,
  const char * changed_slot,
  QObject * receiver)
: RosActionProperty(name, default_value, action_type, description, parent, changed_slot, receiver)
  , filter_(filter)
  , filter_enabled_(true)
{
}

void RosFilteredActionProperty::enableFilter(bool enabled)
{
  filter_enabled_ = enabled;
  fillActionList();
}

QRegExp RosFilteredActionProperty::filter() const
{
  return filter_;
}

void RosFilteredActionProperty::fillActionList()
{
  QStringList filtered_strings_;

  // Obtain list of available actions
  RosActionProperty::fillActionList();
  // Apply filter
  if (filter_enabled_) {
    strings_ = strings_.filter(filter_);
  }
}
}  // end namespace properties
}  // end namespace rviz_common
