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

#include "rclcpp/qos.hpp"

#include "rviz_common/display_context.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/qos_profile_property.hpp"
#include "rviz_common/properties/string_property.hpp"

namespace rviz_default_plugins
{
namespace tools
{

InitialPoseTool::InitialPoseTool()
: qos_profile_(5)
{
  shortcut_key_ = 'p';

  topic_property_ = new rviz_common::properties::StringProperty(
    "Topic", "initialpose",
    "The topic on which to publish initial pose estimates.",
    getPropertyContainer(), SLOT(updateTopic()), this);

  qos_profile_property_ = new rviz_common::properties::QosProfileProperty(
    topic_property_, qos_profile_);

  covariance_x_property_ = new rviz_common::properties::FloatProperty(
    "Covariance x", 0.5f * 0.5f, "Covariance on the x-axis.",
    getPropertyContainer(), 0, this);
  covariance_y_property_ = new rviz_common::properties::FloatProperty(
    "Covariance y", 0.5f * 0.5f, "Covariance on the y-axis.",
    getPropertyContainer(), 0, this);
  covariance_yaw_property_ = new rviz_common::properties::FloatProperty(
    "Covariance yaw", static_cast<float>(M_PI / 12.0 * M_PI / 12.0),
    "Covariance on the yaw-axis.", getPropertyContainer(), 0, this);
}

InitialPoseTool::~InitialPoseTool() = default;

void InitialPoseTool::onInitialize()
{
  PoseTool::onInitialize();
  qos_profile_property_->initialize(
    [this](rclcpp::QoS profile) {this->qos_profile_ = profile;});
  setName("2D Pose Estimate");
  updateTopic();
}

void InitialPoseTool::updateTopic()
{
  // TODO(anhosi, wjwwood): replace with abstraction for publishers once available
  rclcpp::Node::SharedPtr raw_node =
    context_->getRosNodeAbstraction().lock()->get_raw_node();
  publisher_ = raw_node->
    template create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    topic_property_->getStdString(), qos_profile_);
  clock_ = raw_node->get_clock();
}

void InitialPoseTool::onPoseSet(double x, double y, double theta)
{
  std::string fixed_frame = context_->getFixedFrame().toStdString();

  geometry_msgs::msg::PoseWithCovarianceStamped pose;
  pose.header.frame_id = fixed_frame;
  pose.header.stamp = clock_->now();

  pose.pose.pose.position.x = x;
  pose.pose.pose.position.y = y;
  pose.pose.pose.position.z = 0.0;

  pose.pose.pose.orientation = orientationAroundZAxis(theta);

  pose.pose.covariance[6 * 0 + 0] = covariance_x_property_->getFloat();
  pose.pose.covariance[6 * 1 + 1] = covariance_y_property_->getFloat();
  pose.pose.covariance[6 * 5 + 5] = covariance_yaw_property_->getFloat();

  logPose("estimate", pose.pose.pose.position, pose.pose.pose.orientation, theta, fixed_frame);

  publisher_->publish(pose);
}

}  // namespace tools
}  // namespace rviz_default_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::tools::InitialPoseTool, rviz_common::Tool)
