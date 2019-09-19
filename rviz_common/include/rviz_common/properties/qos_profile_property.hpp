/*
 * Copyright (c) 2019, Martin Idel
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
 *     * Neither the name of the copyright holder nor the names of its
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

#ifndef RVIZ_COMMON__PROPERTIES__QOS_PROFILE_PROPERTY_HPP_
#define RVIZ_COMMON__PROPERTIES__QOS_PROFILE_PROPERTY_HPP_

#include <functional>

#include <QObject>  // NOLINT: cpplint is unable to handle the include order here

#include "rclcpp/qos.hpp"

#include "rviz_common/visibility_control.hpp"

namespace rviz_common
{
namespace properties
{
class Property;

class IntProperty;

class EditableEnumProperty;

class RVIZ_COMMON_PUBLIC QosProfileProperty : public QObject
{
  Q_OBJECT

public:
  explicit QosProfileProperty(
    Property * parent_property,
    rclcpp::QoS default_profile = rclcpp::QoS(5)
  );

  /**
   * This function needs to be called after initialization to set the callback for when the
   * property value changes. Note that this is not done in the constructor to allow using member
   * functions of displays.
   *
   * \param qos_changed_callback Function to call when the profile changed
   */
  void initialize(std::function<void(rclcpp::QoS)> qos_changed_callback);

private Q_SLOTS:
  void updateQosProfile();

private:
  rviz_common::properties::IntProperty * depth_property_;
  rviz_common::properties::EditableEnumProperty * history_policy_property_;
  rviz_common::properties::EditableEnumProperty * reliability_policy_property_;
  rviz_common::properties::EditableEnumProperty * durability_policy_property_;
  rclcpp::QoS qos_profile_;
  std::function<void(rclcpp::QoS)> qos_changed_callback_;
};

}  // namespace properties
}  // namespace rviz_common

#endif  // RVIZ_COMMON__PROPERTIES__QOS_PROFILE_PROPERTY_HPP_
