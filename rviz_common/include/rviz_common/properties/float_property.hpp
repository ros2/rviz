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

#ifndef RVIZ_COMMON__PROPERTIES__FLOAT_PROPERTY_HPP_
#define RVIZ_COMMON__PROPERTIES__FLOAT_PROPERTY_HPP_

#include "rviz_common/properties/property.hpp"
#include "rviz_common/visibility_control.hpp"

namespace rviz_common
{
namespace properties
{

/// Property specialized to enforce floating point max/min.
class RVIZ_COMMON_PUBLIC FloatProperty : public Property
{
  Q_OBJECT

public:
  explicit FloatProperty(
    const QString & name = QString(),
    float default_value = 0,
    const QString & description = QString(),
    Property * parent = 0,
    const char * changed_slot = 0,
    QObject * receiver = 0);

  /// Set the new value for this property; return true if different, else false.
  /**
   * If the new value is different from the old value, this emits
   * aboutToChange() before changing the value and changed() after.
   *
   * Overridden from Property::setValue() to enforce minimum and maximum.
   */
  bool setValue(const QVariant & new_value) override;

  /// Get the value of the Property as a float.
  float getFloat() const;

  /// Set the enforced minimum value.
  void setMin(float min);

  /// Get the currently enforced minimum.
  float getMin();

  /// Set the enforced maximum value.
  void setMax(float max);

  /// Get the currently enforced maximum.
  float getMax();

public Q_SLOTS:
  /// Float-typed "SLOT" version of setValue().
  bool setFloat(float new_value);

  /// Add the given @a delta to the property value.
  bool add(float delta);

  /// Multiply the property value by the given factor.
  bool multiply(float factor);

private:
  float min_;
  float max_;
};

}  // namespace properties
}  // namespace rviz_common

#endif  // RVIZ_COMMON__PROPERTIES__FLOAT_PROPERTY_HPP_
