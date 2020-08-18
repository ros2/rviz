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

#include "rviz_common/properties/qos_profile_property.hpp"

#include <functional>
#include <map>
#include <utility>

#include "rclcpp/qos.hpp"

#include "rviz_common/properties/editable_enum_property.hpp"
#include "rviz_common/properties/int_property.hpp"
#include "rviz_common/properties/property.hpp"

namespace rviz_common
{
namespace properties
{

const std::map<rmw_qos_history_policy_t, QString> history_policies {
  {RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT, "System Default"},
  {RMW_QOS_POLICY_HISTORY_KEEP_LAST, "Keep Last"},
  {RMW_QOS_POLICY_HISTORY_KEEP_ALL, "Keep All"},
};

const std::map<rmw_qos_reliability_policy_t, QString> reliability_policies {
  {RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT, "System Default"},
  {RMW_QOS_POLICY_RELIABILITY_RELIABLE, "Reliable"},
  {RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT, "Best Effort"},
};

const std::map<rmw_qos_durability_policy_t, QString> durability_policies {
  {RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT, "System Default"},
  {RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL, "Transient Local"},
  {RMW_QOS_POLICY_DURABILITY_VOLATILE, "Volatile"},
};

QosProfileProperty::QosProfileProperty(Property * parent_property, rclcpp::QoS default_profile)
: qos_profile_(default_profile),
  qos_changed_callback_([](rclcpp::QoS profile) {(void) profile;})
{
  depth_property_ = new IntProperty(
    "Depth", static_cast<int>(default_profile.get_rmw_qos_profile().depth),
    "Set the depth of the incoming message queue. Increasing this is useful if your "
    "incoming TF data is delayed significantly from your message data, but it can greatly "
    "increase memory usage if the messages are big.",
    parent_property, SLOT(updateQosProfile()), this, 1, INT_MAX);

  history_policy_property_ = new EditableEnumProperty(
    "History Policy", history_policies.at(default_profile.get_rmw_qos_profile().history),
    "Set the history policy: 'Keep all' will keep every message and ignore the depth, while "
    "'keep last' will only keep the last messages up to the depth.",
    parent_property, SLOT(updateQosProfile()), this);
  for (const auto & history : history_policies) {
    history_policy_property_->addOption(history.second);
  }

  reliability_policy_property_ = new EditableEnumProperty(
    "Reliability Policy",
    reliability_policies.at(default_profile.get_rmw_qos_profile().reliability),
    "Set the reliability policy: When choosing 'Best effort', no guarantee will be given that the "
    "messages will be delivered, choosing 'Reliable', messages will be sent until received.",
    parent_property, SLOT(updateQosProfile()), this);
  for (const auto & reliability : reliability_policies) {
    reliability_policy_property_->addOption(reliability.second);
  }

  durability_policy_property_ = new EditableEnumProperty(
    "Durability Policy", durability_policies.at(default_profile.get_rmw_qos_profile().durability),
    "Set the durability policy: When choosing 'Transient Local' the publisher will be responsible "
    "for persisting data for late joining subscribers, choosing 'Volatile', no attempt at "
    "persistence will be made.",
    parent_property, SLOT(updateQosProfile()), this);
  for (const auto & durability : durability_policies) {
    durability_policy_property_->addOption(durability.second);
  }

  updateQosProfile();
}

template<class T>
T get_profile(std::map<T, QString> enum_map, EditableEnumProperty * enum_property, T default_entry)
{
  for (const auto & entries : enum_map) {
    if (entries.second == enum_property->getString()) {
      return entries.first;
    }
  }
  return default_entry;
}

void QosProfileProperty::initialize(std::function<void(rclcpp::QoS)> qos_changed_callback)
{
  qos_changed_callback_ = std::move(qos_changed_callback);
}

void QosProfileProperty::updateQosProfile()
{
  rmw_qos_profile_t profile = rmw_qos_profile_default;
  profile.depth = depth_property_->getInt();

  profile.history = get_profile(history_policies, history_policy_property_, profile.history);
  profile.reliability = get_profile(
    reliability_policies, reliability_policy_property_, profile.reliability);
  profile.durability = get_profile(
    durability_policies, durability_policy_property_, profile.durability);
  qos_changed_callback_(rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(profile), profile));
}

}  // namespace properties
}  // namespace rviz_common
