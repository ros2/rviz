// Copyright (c) 2026, Dylan Gallagher.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/subscription_options.hpp"
#include "rmw/qos_policy_kind.h"
#include "rviz_common/message_filter_display.hpp"
#include "rviz_common/properties/status_property.hpp"

using testing::HasSubstr;

class TestMessageFilterDisplay
  : public rviz_common::MessageFilterDisplay<geometry_msgs::msg::PoseStamped>
{
public:
  rclcpp::SubscriptionOptions subscriptionOptions()
  {
    return createSubscriptionOptions(rclcpp::get_logger("message_filter_display_test"), "/pose");
  }

  void receiveMessage(const geometry_msgs::msg::PoseStamped::ConstSharedPtr & message)
  {
    processTypeErasedMessage(std::static_pointer_cast<const void>(message));
  }

  void setStatus(
    rviz_common::properties::StatusProperty::Level level,
    const QString & name,
    const QString & text) override
  {
    status_level_ = level;
    status_name_ = name;
    status_text_ = text;
  }

  rviz_common::properties::StatusProperty::Level status_level_ =
    rviz_common::properties::StatusProperty::Ok;
  QString status_name_;
  QString status_text_;
  size_t processed_messages_ = 0;

protected:
  void processMessage(geometry_msgs::msg::PoseStamped::ConstSharedPtr) override
  {
    ++processed_messages_;
  }
};

TEST(MessageFilterDisplay, incompatible_qos_updates_topic_status_and_compatible_message_recovers)
{
  TestMessageFilterDisplay display;
  auto options = display.subscriptionOptions();
  ASSERT_TRUE(options.event_callbacks.incompatible_qos_callback);

  rclcpp::QOSRequestedIncompatibleQoSInfo info{};
  info.total_count = 1;
  info.total_count_change = 1;
  info.last_policy_kind = RMW_QOS_POLICY_RELIABILITY;
  options.event_callbacks.incompatible_qos_callback(info);

  EXPECT_EQ(rviz_common::properties::StatusProperty::Error, display.status_level_);
  EXPECT_EQ("Topic", display.status_name_);
  EXPECT_THAT(display.status_text_.toStdString(), HasSubstr("Incompatible QoS"));
  EXPECT_THAT(display.status_text_.toStdString(), HasSubstr("RELIABILITY_QOS_POLICY"));
  EXPECT_EQ(0u, display.processed_messages_);

  display.receiveMessage(std::make_shared<const geometry_msgs::msg::PoseStamped>());

  EXPECT_EQ(rviz_common::properties::StatusProperty::Ok, display.status_level_);
  EXPECT_EQ("Topic", display.status_name_);
  EXPECT_THAT(display.status_text_.toStdString(), HasSubstr("1 messages received"));
  EXPECT_EQ(1u, display.processed_messages_);
}
