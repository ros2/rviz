/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

#include "rviz_default_plugins/tools/nav_goal/goal_tool.hpp"

#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "rviz_common/display_context.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/properties/string_property.hpp"

namespace rviz_default_plugins
{
namespace tools
{

GoalTool::GoalTool()
: rviz_default_plugins::tools::PoseTool()
{
  shortcut_key_ = 'g';

  topic_property_ = new rviz_common::properties::StringProperty("Topic", "goal",
      "The topic on which to publish navigation goals.",
      getPropertyContainer(), SLOT(updateTopic()), this);
}

GoalTool::~GoalTool() = default;

void GoalTool::onInitialize()
{
  PoseTool::onInitialize();
  setName("2D Nav Goal");
  updateTopic();
}

void GoalTool::updateTopic()
{
  // TODO(anhosi, wjwwood): replace with abstraction for publishers once available
  publisher_ = context_->getRosNodeAbstraction().lock()->get_raw_node()->
    template create_publisher<geometry_msgs::msg::PoseStamped>(topic_property_->getStdString());
}

void GoalTool::onPoseSet(double x, double y, double theta)
{
  std::string fixed_frame = context_->getFixedFrame().toStdString();

  geometry_msgs::msg::PoseStamped goal;
  goal.header.stamp = rclcpp::Clock().now();
  goal.header.frame_id = fixed_frame;

  goal.pose.position.x = x;
  goal.pose.position.y = y;
  goal.pose.position.z = 0.0;

  // set quaternion from yaw only
  goal.pose.orientation.x = 0.0;
  goal.pose.orientation.y = 0.0;
  goal.pose.orientation.z = sin(theta) / (2 * cos(theta / 2));
  goal.pose.orientation.w = cos(theta / 2);

  RVIZ_COMMON_LOG_INFO_STREAM("Setting goal: Frame:" << fixed_frame << ", Position(" <<
    goal.pose.position.x << ", " << goal.pose.position.y << ", " << goal.pose.position.z <<
    "), Orientation(" << goal.pose.orientation.x << ", " << goal.pose.orientation.y << ", " <<
    goal.pose.orientation.z << ", " << goal.pose.orientation.w << ") = Angle: " << theta);

  publisher_->publish(goal);
}

}  // namespace tools
}  // namespace rviz_default_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::tools::GoalTool, rviz_common::Tool)
