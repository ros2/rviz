// Copyright (c) 2026, Open Source Robotics Foundation, Inc.
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

#ifndef RVIZ_COMMON__PROPERTIES__ROS_TOPIC_MULTI_TYPE_PROPERTY_HPP_
#define RVIZ_COMMON__PROPERTIES__ROS_TOPIC_MULTI_TYPE_PROPERTY_HPP_

#include <QSet>
#include <QString>

#include "rviz_common/properties/ros_topic_property.hpp"
#include "rviz_common/visibility_control.hpp"

namespace rviz_common
{
namespace properties
{

// Like RosTopicProperty but can accept multiple message types
class RVIZ_COMMON_PUBLIC RosTopicMultiTypeProperty : public RosTopicProperty
{
  Q_OBJECT

public:
  explicit RosTopicMultiTypeProperty(
    const QString & name = QString(), const QString & default_value = QString(),
    const QSet<QString> & message_types = QSet<QString>(),
    const QString & description = QString(), Property * parent = nullptr,
    const char * changed_slot = nullptr, QObject * receiver = nullptr)
  : RosTopicProperty(name, default_value, "", description, parent, changed_slot, receiver),
    message_types_(message_types)
  {
  }

  void setMessageTypes(const QSet<QString> & message_types)
  {
    message_types_ = message_types;
  }

  QSet<QString> getMessageTypes() const {return message_types_;}

protected Q_SLOTS:
  void fillTopicList() override;

private:
  // Hide the parent class methods which only take a single type
  using RosTopicProperty::getMessageType;
  using RosTopicProperty::setMessageType;

  // Instead of one message type, store a list of allowed types
  QSet<QString> message_types_;
};

}  // end namespace properties
}  // end namespace rviz_common

#endif  // RVIZ_COMMON__PROPERTIES__ROS_TOPIC_MULTI_TYPE_PROPERTY_HPP_
