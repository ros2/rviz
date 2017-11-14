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

#include "rviz_common/properties/float_property.hpp"

#include <cfloat>  // for FLT_MAX

#include <QtGlobal>  // NOLINT: cpplint is unable to handle the include order here

namespace rviz_common
{
namespace properties
{

FloatProperty::FloatProperty(
  const QString & name,
  float default_value,
  const QString & description,
  Property * parent,
  const char * changed_slot,
  QObject * receiver)
: Property(name, default_value, description, parent, changed_slot, receiver),
  min_(-FLT_MAX),
  max_(FLT_MAX)
{
}

bool FloatProperty::setValue(const QVariant & new_value)
{
  return Property::setValue(qBound(min_, new_value.toFloat(), max_));
}

float FloatProperty::getFloat() const
{
  return getValue().toFloat();
}

void FloatProperty::setMin(float min)
{
  min_ = min;
  setValue(getValue());
}

float FloatProperty::getMin()
{
  return min_;
}

void FloatProperty::setMax(float max)
{
  max_ = max;
  setValue(getValue());
}

float FloatProperty::getMax()
{
  return max_;
}

bool FloatProperty::setFloat(float new_value)
{
  return setValue(new_value);
}

bool FloatProperty::add(float delta)
{
  return setValue(delta + getValue().toFloat());
}

bool FloatProperty::multiply(float factor)
{
  return setValue(factor * getValue().toFloat());
}

}  // end namespace properties
}  // end namespace rviz_common
