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

#ifndef RVIZ_DEFAULT_PLUGINS__TOOLS__POSE_ESTIMATE__INITIAL_POSE_TOOL_HPP_
#define RVIZ_DEFAULT_PLUGINS__TOOLS__POSE_ESTIMATE__INITIAL_POSE_TOOL_HPP_

#include <QObject>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/qos.hpp"

#include "rviz_default_plugins/tools/pose/pose_tool.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

namespace rviz_common
{
class DisplayContext;
namespace properties
{
class FloatProperty;
class StringProperty;
class QosProfileProperty;
}  // namespace properties
}  // namespace rviz_common

namespace rviz_default_plugins
{
namespace tools
{
class RVIZ_DEFAULT_PLUGINS_PUBLIC InitialPoseTool : public PoseTool
{
  Q_OBJECT

public:
  InitialPoseTool();

  ~InitialPoseTool() override;

  void onInitialize() override;

protected:
  void onPoseSet(double x, double y, double theta) override;

private Q_SLOTS:
  void updateTopic();

private:
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
  rclcpp::Clock::SharedPtr clock_;

  rviz_common::properties::StringProperty * topic_property_;
  rviz_common::properties::QosProfileProperty * qos_profile_property_;
  rviz_common::properties::FloatProperty * covariance_x_property_;
  rviz_common::properties::FloatProperty * covariance_y_property_;
  rviz_common::properties::FloatProperty * covariance_yaw_property_;

  rclcpp::QoS qos_profile_;
};

}  // namespace tools
}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__TOOLS__POSE_ESTIMATE__INITIAL_POSE_TOOL_HPP_
