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

#ifndef RVIZ_COMMON__PROPERTIES__BOOL_PROPERTY_HPP_
#define RVIZ_COMMON__PROPERTIES__BOOL_PROPERTY_HPP_

#include "rviz_common/properties/property.hpp"
#include "rviz_common/visibility_control.hpp"

namespace rviz_common
{
namespace properties
{

/// Property specialized to provide getter for booleans.
class RVIZ_COMMON_PUBLIC BoolProperty : public Property
{
  Q_OBJECT

public:
  explicit BoolProperty(
    const QString & name = QString(),
    bool default_value = false,
    const QString & description = QString(),
    Property * parent = nullptr,
    const char * changed_slot = nullptr,
    QObject * receiver = nullptr);

  ~BoolProperty() override;

  /// Return the value of this.
  virtual bool getBool() const;

  /// If given true, disable this properties own children when this has a value of false.
  void setDisableChildrenIfFalse(bool disable);

  /// Return the disabled children if this has a value of false.
  bool getDisableChildrenIfFalse();

  /// Return the disabled children.
  bool getDisableChildren() override;

public Q_SLOTS:
  /// Set the value of this.
  bool setBool(bool value);

private:
  bool disable_children_if_false_;
};

}  // namespace properties
}  // namespace rviz_common

#endif  // RVIZ_COMMON__PROPERTIES__BOOL_PROPERTY_HPP_
