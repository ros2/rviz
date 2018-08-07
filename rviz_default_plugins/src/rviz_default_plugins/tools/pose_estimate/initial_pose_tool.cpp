/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

#define _USE_MATH_DEFINES
#include "rviz_default_plugins/tools/pose_estimate/initial_pose_tool.hpp"

#include <string>

#include "rviz_common/display_context.hpp"
#include "rviz_common/properties/string_property.hpp"
#include "rviz_common/logging.hpp"

namespace rviz_default_plugins
{
namespace tools
{

InitialPoseTool::InitialPoseTool()
{
  shortcut_key_ = 'p';

  topic_property_ = new rviz_common::properties::StringProperty("Topic", "initialpose",
      "The topic on which to publish initial pose estimates.",
      getPropertyContainer(), SLOT(updateTopic()), this);
}

InitialPoseTool::~InitialPoseTool() = default;

void InitialPoseTool::onInitialize()
{
  PoseTool::onInitialize();
  setName("2D Pose Estimate");
  updateTopic();
}

void InitialPoseTool::updateTopic()
{
  // TODO(anhosi, wjwwood): replace with abstraction for publishers once available
  publisher_ = context_->getRosNodeAbstraction().lock()->get_raw_node()->
    template create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    topic_property_->getStdString());
}

void InitialPoseTool::onPoseSet(double x, double y, double theta)
{
  std::string fixed_frame = context_->getFixedFrame().toStdString();

  geometry_msgs::msg::PoseWithCovarianceStamped pose;
  pose.header.frame_id = fixed_frame;
  pose.header.stamp = rclcpp::Clock().now();

  pose.pose.pose.position.x = x;
  pose.pose.pose.position.y = y;
  pose.pose.pose.position.z = 0.0;

  pose.pose.pose.orientation = orientationAroundZAxis(theta);

  pose.pose.covariance[6 * 0 + 0] = 0.5 * 0.5;
  pose.pose.covariance[6 * 1 + 1] = 0.5 * 0.5;
  pose.pose.covariance[6 * 5 + 5] = M_PI / 12.0 * M_PI / 12.0;

  logPose(pose.pose.pose.position, pose.pose.pose.orientation, theta, fixed_frame);

  publisher_->publish(pose);
}

}  // namespace tools
}  // namespace rviz_default_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::tools::InitialPoseTool, rviz_common::Tool)
