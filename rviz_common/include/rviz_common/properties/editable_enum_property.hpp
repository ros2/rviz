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

#ifndef RVIZ_COMMON__PROPERTIES__EDITABLE_ENUM_PROPERTY_HPP_
#define RVIZ_COMMON__PROPERTIES__EDITABLE_ENUM_PROPERTY_HPP_

#include <string>

#include <QString>  // NOLINT: cpplint is unable to handle the include order here
#include <QStringList>  // NOLINT: cpplint is unable to handle the include order here
#include <QWidget>  // NOLINT: cpplint is unable to handle the include order here

#include "rviz_common/properties/string_property.hpp"
#include "rviz_common/visibility_control.hpp"

namespace rviz_common
{
namespace properties
{

/// Editable Enum property.
/**
 * An editable enum property works like a string property, but with
 * the addition of a drop-down list of predefined choices.
 */
class RVIZ_COMMON_PUBLIC EditableEnumProperty : public StringProperty
{
  Q_OBJECT

public:
  EditableEnumProperty(
    const QString & name = QString(),
    const QString & default_value = QString(),
    const QString & description = QString(),
    Property * parent = 0,
    const char * changed_slot = 0,
    QObject * receiver = 0);

  /// Clear options.
  virtual void clearOptions();

  /// Add an option from a QString.
  virtual void addOption(const QString & option);

  /// Add an option from a std::string.
  void addOptionStd(const std::string & option);

  /// Create the editor.
  virtual QWidget * createEditor(QWidget * parent, const QStyleOptionViewItem & option);
  // virtual QWidget * createEditor(QWidget * parent);

  /// Sort the option strings.
  void sortOptions();

public Q_SLOTS:
  /// Set the string value from a QString.
  virtual void setString(const QString & str);

Q_SIGNALS:
  /// requestOptions() is emitted each time createEditor() is called.
  /**
   * A connection to this signal should never be made with a queued connection,
   * because then the "emit" would return before the changes to the options in
   * the EnumProperty were made.
   *
   * A connected slot should make calls to clearOptions() and/or addOption() as
   * needed.
   * The option list in the EditableEnumProperty will not be cleared before the
   * signal is emitted.
   */
  void requestOptions(EditableEnumProperty * property_in_need_of_options);

protected:
  QStringList strings_;
};

}  // namespace properties
}  // namespace rviz_common

#endif  // RVIZ_COMMON__PROPERTIES__EDITABLE_ENUM_PROPERTY_HPP_
