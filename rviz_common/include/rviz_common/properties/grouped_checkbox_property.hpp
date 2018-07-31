/*
 * Copyright (c) 2018, Bosch Software Innovations GmbH.
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

#ifndef RVIZ_COMMON__PROPERTIES__GROUPED_CHECKBOX_PROPERTY_HPP_
#define RVIZ_COMMON__PROPERTIES__GROUPED_CHECKBOX_PROPERTY_HPP_

#include <memory>

#include "rviz_common/properties/bool_property.hpp"

namespace rviz_common
{
namespace properties
{
class GroupedCheckboxPropertyGroup;

/** @brief Checkbox property that is grouped together with other checkbox properties.
 *
 * Behaves like radio buttons (exclusive selection, no manual deselect)
 */
class GroupedCheckboxProperty : public BoolProperty
{
public:
  explicit
  GroupedCheckboxProperty(
    std::shared_ptr<GroupedCheckboxPropertyGroup> group,
    const QString & name = QString(),
    bool default_value = false,
    const QString & description = QString(),
    Property * parent = nullptr,
    const char * changed_slot = nullptr,
    QObject * receiver = nullptr);

  /**
   * Sets the value of this property to true
   * and triggers the group to disable all other GroupedCheckboxProperty.
   *
   * @param new_value IGNORED, radio buttons can generally not be deselected manually
   * @return
   */
  bool setValue(const QVariant & new_value) override;

  /// Sets the value of the underlying property without triggering the group
  bool setRawValue(const QVariant & new_value);

private:
  std::shared_ptr<GroupedCheckboxPropertyGroup> group_;
};

}  // namespace properties
}  // namespace rviz_common

#endif  // RVIZ_COMMON__PROPERTIES__GROUPED_CHECKBOX_PROPERTY_HPP_
