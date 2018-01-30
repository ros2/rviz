/*
 * Copyright (c) 2009, Willow Garage, Inc.
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

#ifndef RVIZ_COMMON__PROPERTIES__DISPLAY_VISIBILITY_PROPERTY_HPP_
#define RVIZ_COMMON__PROPERTIES__DISPLAY_VISIBILITY_PROPERTY_HPP_

#include <cstdint>
#include <map>
#include <string>

#include <QObject>  // NOLINT: cpplint cannot handle the include order here
#include <QString>  // NOLINT: cpplint cannot handle the include order here

#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/visibility_control.hpp"

namespace rviz_common
{

class DisplayContext;

class Display;

namespace properties
{
class Property;

class BoolProperty;

class DisplayVisibilityProperty;

/*
 * @brief Changes one visibility bit of a given Display
 */
class RVIZ_COMMON_PUBLIC DisplayVisibilityProperty : public BoolProperty
{
  Q_OBJECT

public:
  DisplayVisibilityProperty(
    uint32_t vis_bit,
    Display * display,
    const QString & name = QString(),
    bool default_value = false,
    const QString & description = QString(),
    Property * parent = 0,
    const char * changed_slot = 0,
    QObject * receiver = 0);

  ~DisplayVisibilityProperty() override;

  virtual void update();

  bool getBool() const override;

  Qt::ItemFlags getViewFlags(int column) const override;

public Q_SLOTS:
  bool setValue(const QVariant & new_value) override;

protected:
  uint32_t vis_bit_;
  Display * display_;
  bool custom_name_;
};

}  // namespace properties

}  // namespace rviz_common

#endif  // RVIZ_COMMON__PROPERTIES__DISPLAY_VISIBILITY_PROPERTY_HPP_
