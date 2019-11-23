/*
 * Copyright (c) 2018, Bosch Software Innovations GmbH.
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
 *     * Neither the name of the copyright holder nor the names of its contributors
 *       may be used to endorse or promote products derived from
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

// Derived from odometry publisher

#ifndef RVIZ_DEFAULT_PLUGINS__PUBLISHERS__POSE_WITH_COVARIANCE_PUBLISHER_HPP_
#define RVIZ_DEFAULT_PLUGINS__PUBLISHERS__POSE_WITH_COVARIANCE_PUBLISHER_HPP_

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;  // NOLINT

namespace nodes
{

class PoseWithCovariancePublisher : public rclcpp::Node
{
public:
  PoseWithCovariancePublisher()
  : Node("pose_with_covariance_publisher")
  {
    publisher = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "pose_with_covariance", 10);
    timer = this->create_wall_timer(
      200ms, std::bind(&PoseWithCovariancePublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto pose_with_covariance_stamped = geometry_msgs::msg::PoseWithCovarianceStamped();
    pose_with_covariance_stamped.header.frame_id = "pose_with_covariance_frame";
    pose_with_covariance_stamped.header.stamp = rclcpp::Clock().now();

    pose_with_covariance_stamped.pose.pose.position.x = 0.0f;
    pose_with_covariance_stamped.pose.pose.position.y = 0.0f;
    pose_with_covariance_stamped.pose.pose.position.z = 0.0f;
    pose_with_covariance_stamped.pose.pose.orientation.x = 1;
    pose_with_covariance_stamped.pose.pose.orientation.y = 0;
    pose_with_covariance_stamped.pose.pose.orientation.z = 0;
    pose_with_covariance_stamped.pose.pose.orientation.w = 0;
    pose_with_covariance_stamped.pose.covariance = std::array<double, 36>{
      {0.75, 0.04, 0.1, 0, 0, 0,
        0.04, 0.7, 0.4, 0, 0, 0,
        0.1, 0.4, 0.5, 0, 0, 0,
        0, 0, 0, 0.8, 0.25, 0.06,
        0, 0, 0, 0.25, 0.3, 0.22,
        0, 0, 0, 0.06, 0.22, 0.6}
    };

    publisher->publish(pose_with_covariance_stamped);
  }

  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher;
};


}  // namespace nodes

#endif  // RVIZ_DEFAULT_PLUGINS__PUBLISHERS__POSE_WITH_COVARIANCE_PUBLISHER_HPP_
