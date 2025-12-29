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
#include <QRegularExpression>  // NOLINT: cpplint can't handle Qt imports
#include <QString>  // NOLINT: cpplint can't handle Qt imports
#include <QStringList>  // NOLINT: cpplint can't handle Qt imports

#include "rviz_common/properties/ros_service_property.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"

namespace rviz_common
{
namespace properties
{

RosServiceProperty::RosServiceProperty(
  const QString & name,
  const QString & default_value,
  const QString & service_type,
  const QString & description,
  Property * parent,
  const char * changed_slot,
  QObject * receiver)
: EditableEnumProperty(name, default_value, description, parent, changed_slot, receiver),
  rviz_ros_node_(),
  service_type_(service_type)
{
  connect(
    this, SIGNAL(requestOptions(EditableEnumProperty*)),
    this, SLOT(fillServiceList()));
}

void RosServiceProperty::initialize(ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node)
{
  rviz_ros_node_ = rviz_ros_node;
}

void RosServiceProperty::setServiceType(const QString & service_type)
{
  service_type_ = service_type;
}

QString RosServiceProperty::getServiceType() const
{
  return service_type_;
}

QString RosServiceProperty::getService() const
{
  return getValue().toString();
}

std::string RosServiceProperty::getServiceStd() const
{
  return getValue().toString().toStdString();
}

bool RosServiceProperty::isEmpty() const
{
  return getServiceStd().empty();
}

void RosServiceProperty::fillServiceList()
{
  QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));
  clearOptions();

  std::string std_service_type = service_type_.toStdString();
  std::map<std::string, std::vector<std::string>> service_servers =
    rviz_ros_node_.lock()->get_service_names_and_types();

  for (const auto & service : service_servers) {
    // Only add service whose type matches.
    for (const auto & type : service.second) {
      if (type == std_service_type) {
        addOptionStd(service.first);
      }
    }
  }
  sortOptions();
  QApplication::restoreOverrideCursor();
}

RosFilteredServiceProperty::RosFilteredServiceProperty(
  const QString & name,
  const QString & default_value,
  const QString & service_type,
  const QString & description,
  const QRegularExpression & filter,
  Property * parent,
  const char * changed_slot,
  QObject * receiver)
: RosServiceProperty(name, default_value, service_type, description, parent, changed_slot, receiver)
  , filter_(filter)
  , filter_enabled_(true)
{
}

void RosFilteredServiceProperty::enableFilter(bool enabled)
{
  filter_enabled_ = enabled;
  fillServiceList();
}

QRegularExpression RosFilteredServiceProperty::filter() const
{
  return filter_;
}

void RosFilteredServiceProperty::fillServiceList()
{
  QStringList filtered_strings_;

  // Obtain list of available services
  RosServiceProperty::fillServiceList();
  // Apply filter
  if (filter_enabled_) {
    strings_ = strings_.filter(filter_);
  }
}
}  // end namespace properties
}  // end namespace rviz_common
