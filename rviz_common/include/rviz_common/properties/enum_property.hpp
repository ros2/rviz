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

#ifndef RVIZ_COMMON__PROPERTIES__ENUM_PROPERTY_HPP_
#define RVIZ_COMMON__PROPERTIES__ENUM_PROPERTY_HPP_

#include <QStringList>

#include <string>

#include "rviz_common/properties/string_property.hpp"
#include "rviz_common/visibility_control.hpp"

namespace rviz_common
{
namespace properties
{

/// Enum property.
/**
 * An enum property works like a string property all the way through
 * the system property system, except when you get a changed() signal
 * you can call getOptionInt() to get the integer value of the current
 * option.
 * The integer returned will be that passed to addOption() for with the
 * string that is currently selected.
 */
class RVIZ_COMMON_PUBLIC EnumProperty : public StringProperty
{
  Q_OBJECT

public:
  explicit
  EnumProperty(
    const QString & name = QString(),
    const QString & default_value = QString(),
    const QString & description = QString(),
    Property * parent = 0,
    const char * changed_slot = 0,
    QObject * receiver = 0);

  /// Clear the list of options.
  /**
   * Does not change the current value of the property.
   */
  virtual
  void
  clearOptions();

  virtual
  void
  addOption(const QString & option, int value = 0);

  void
  addOptionStd(const std::string & option, int value = 0);

  /// Return the int value of the currently-chosen option, or 0 if the current value is not an int.
  virtual int getOptionInt();

  QWidget * createEditor(QWidget * parent, const QStyleOptionViewItem & option) override;

public Q_SLOTS:
  /// Set the value of this property to the given string.
  /**
   * Does not force the value to be one of the list of options.
   */
  virtual void setString(const QString & str);

  /// Set the value of this property to the given std::string.
  /**
   * Does not force the value to be one of the list of options.
   */
  void setStringStd(const std::string & str);

  /// Sort the option strings.
  /**
   * Does not change string/int associations.
   */
  void sortOptions();

Q_SIGNALS:
  /// requestOptions() is emitted each time createEditor() is called.
  /**
   * A connection to this signal should never be made with a queued
   * connection, because then the "emit" would return before the
   * changes to the options in the EnumProperty were made.
   *
   * A connected slot should make calls to clearOptions() and/or
   * addOption() as needed.  The option list in the EnumProperty will
   * not be cleared before the signal is emitted.
   */
  void requestOptions(rviz_common::properties::EnumProperty * property_in_need_of_options);

private:
  QStringList strings_;
  QHash<QString, int> ints_;
};

}  // namespace properties
}  // namespace rviz_common

#endif  // RVIZ_COMMON__PROPERTIES__ENUM_PROPERTY_HPP_
