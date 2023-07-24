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
#ifndef RVIZ_COMMON__PROPERTIES__ROS_TOPIC_PROPERTY_HPP_
#define RVIZ_COMMON__PROPERTIES__ROS_TOPIC_PROPERTY_HPP_

#include <string>

#include "rviz_common/properties/editable_enum_property.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"
#include "rviz_common/visibility_control.hpp"

namespace rviz_common
{
namespace properties
{

class RVIZ_COMMON_PUBLIC RosTopicProperty : public EditableEnumProperty
{
  Q_OBJECT

public:
  explicit RosTopicProperty(
    const QString & name = QString(),
    const QString & default_value = QString(),
    const QString & message_type = QString(),
    const QString & description = QString(),
    Property * parent = nullptr,
    const char * changed_slot = nullptr,
    QObject * receiver = nullptr);

  void initialize(ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node);

  void setMessageType(const QString & message_type);

  QString getMessageType() const
  {return message_type_;}

  QString getTopic() const
  {return getValue().toString();}

  std::string getTopicStd() const
  {return getValue().toString().toStdString();}

  bool isEmpty() const
  {return getTopicStd().empty();}

protected Q_SLOTS:
  virtual void fillTopicList();

private:
  ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node_;
  QString message_type_;
};

class RVIZ_COMMON_PUBLIC RosFilteredTopicProperty
  : public rviz_common::properties::RosTopicProperty
{
  Q_OBJECT

public:
  RosFilteredTopicProperty(
    const QString & name = QString(),
    const QString & default_value = QString(),
    const QString & message_type = QString(),
    const QString & description = QString(),
    const QRegExp & filter = QRegExp(),
    Property * parent = 0,
    const char * changed_slot = 0,
    QObject * receiver = 0);

  void enableFilter(bool enabled);

  QRegExp filter() const;

protected Q_SLOTS:
  void fillTopicList() override;

private:
  QRegExp filter_;
  bool filter_enabled_;
};

}  // end namespace properties
}  // end namespace rviz_common

#endif  // RVIZ_COMMON__PROPERTIES__ROS_TOPIC_PROPERTY_HPP_
