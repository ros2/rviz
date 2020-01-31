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

#include <gmock/gmock.h>
#include <memory>
#include <string>

#include "rclcpp/qos.hpp"

#include "rviz_common/properties/qos_profile_property.hpp"
#include "rviz_common/properties/property.hpp"
#include "rviz_common/properties/editable_enum_property.hpp"
#include "rviz_common/properties/int_property.hpp"

using namespace ::testing;  // NOLINT

TEST(QosProfilePropertyTest, all_properties_are_found_via_parent) {
  rviz_common::properties::Property parent;
  auto profile_property = std::make_unique<rviz_common::properties::QosProfileProperty>(&parent);

  EXPECT_THAT(parent.childAt(0)->getName().toStdString(), StrEq("Depth"));
  EXPECT_THAT(parent.childAt(1)->getName().toStdString(), StrEq("History Policy"));
  EXPECT_THAT(parent.childAt(2)->getName().toStdString(), StrEq("Reliability Policy"));
  EXPECT_THAT(parent.childAt(3)->getName().toStdString(), StrEq("Durability Policy"));
}

TEST(QosProfilePropertyTest, enum_properties_set_enum_members_of_profile) {
  rclcpp::QoS qos_profile(5);
  bool called = false;
  rviz_common::properties::Property parent;
  auto profile_property = std::make_unique<rviz_common::properties::QosProfileProperty>(&parent);
  profile_property->initialize(
    [&called, &qos_profile](rclcpp::QoS profile) {
      called = true;
      qos_profile = profile;
    });

  parent.childAt(1)->setValue("Keep All");
  parent.childAt(2)->setValue("Best Effort");
  parent.childAt(3)->setValue("Volatile");

  EXPECT_THAT(called, IsTrue());
  EXPECT_THAT(
    qos_profile.get_rmw_qos_profile().history,
    Eq(RMW_QOS_POLICY_HISTORY_KEEP_ALL));
  EXPECT_THAT(
    qos_profile.get_rmw_qos_profile().reliability,
    Eq(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT));
  EXPECT_THAT(
    qos_profile.get_rmw_qos_profile().durability,
    Eq(RMW_QOS_POLICY_DURABILITY_VOLATILE));
}

TEST(QosProfilePropertyTest, depth_is_set_by_property) {
  rclcpp::QoS qos_profile(5);
  bool called = false;
  rviz_common::properties::Property parent;
  auto profile_property = std::make_unique<rviz_common::properties::QosProfileProperty>(&parent);
  profile_property->initialize(
    [&called, &qos_profile](rclcpp::QoS profile) {
      called = true;
      qos_profile = profile;
    });

  parent.childAt(0)->setValue(14);

  EXPECT_THAT(called, IsTrue());
  EXPECT_THAT(qos_profile.get_rmw_qos_profile().depth, Eq(14u));
}
