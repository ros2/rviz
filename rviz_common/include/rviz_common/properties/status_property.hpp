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

#ifndef RVIZ_COMMON__PROPERTIES__STATUS_PROPERTY_HPP_
#define RVIZ_COMMON__PROPERTIES__STATUS_PROPERTY_HPP_

#include <QIcon>

#include "rviz_common/properties/property.hpp"
#include "rviz_common/visibility_control.hpp"

namespace rviz_common
{
namespace properties
{

class RVIZ_COMMON_PUBLIC StatusProperty : public Property
{
  Q_OBJECT

public:
  enum Level { Ok = 0, Warn = 1, Error = 2 };  // values index into status_colors_ array.

  StatusProperty(const QString & name, const QString & text, Level level, Property * parent);

  /// Set the status text.
  bool setValue(const QVariant & new_value) override;  // Overridden from Property.

  /// Return data appropriate for the given column (0 or 1) and role for this StatusProperty.
  QVariant getViewData(int column, int role) const override;

  /// Return item flags appropriate for the given column (0 or 1) for this StatusProperty.
  Qt::ItemFlags getViewFlags(int column) const override;

  /// Return the color appropriate for the given status level.
  static QColor statusColor(Level level);

  /// Return the word appropriate for the given status level: "Ok", "Warn", or "Error".
  static QString statusWord(Level level);

  /// Get the status icon.
  QIcon statusIcon(Level level) const;

  /// Set the status level.
  virtual void setLevel(Level level);

  /// Get the status level.
  virtual Level getLevel() const;

protected:
  Level level_;

private:
  static QColor status_colors_[3];
  static QString status_words_[3];
  QIcon status_icons_[3];
};

typedef StatusProperty::Level StatusLevel;

}  // namespace properties
}  // namespace rviz_common

#endif  // RVIZ_COMMON__PROPERTIES__STATUS_PROPERTY_HPP_
