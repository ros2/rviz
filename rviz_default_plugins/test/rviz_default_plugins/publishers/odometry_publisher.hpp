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

#ifndef RVIZ_DEFAULT_PLUGINS__PUBLISHERS__ODOMETRY_PUBLISHER_HPP_
#define RVIZ_DEFAULT_PLUGINS__PUBLISHERS__ODOMETRY_PUBLISHER_HPP_

#define _USE_MATH_DEFINES
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;  // NOLINT

namespace nodes
{

class OdometryPublisher : public rclcpp::Node
{
public:
  OdometryPublisher()
  : Node("odometry_publisher")
  {
    publisher = this->create_publisher<nav_msgs::msg::Odometry>("odometry", 10);
    timer = this->create_wall_timer(200ms, std::bind(&OdometryPublisher::timer_callback, this));
  }

  // TODO(Martin-Idel-SI): shared_from_this() cannot be used in constructor. Use different
  // constructor once available.
  void initialize()
  {
    broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());
  }

private:
  void timer_callback()
  {
    std::string base_frame("odometry_frame");
    std::string child_frame("map");
    auto now = rclcpp::Clock().now();
    auto id = tf2::Quaternion::getIdentity();

    auto transform = geometry_msgs::msg::TransformStamped();
    transform.header.frame_id = base_frame;
    transform.header.stamp = now;
    transform.child_frame_id = child_frame;
    transform.transform.translation.x = 0.0f;
    transform.transform.translation.y = 0.0f;
    transform.transform.translation.z = 0.0f;
    transform.transform.rotation.x = id.getX();
    transform.transform.rotation.y = id.getY();
    transform.transform.rotation.z = id.getZ();
    transform.transform.rotation.w = id.getW();

    if (broadcaster) {
      broadcaster->sendTransform(transform);
    }

    auto odometry_msg = nav_msgs::msg::Odometry();
    odometry_msg.header.frame_id = base_frame;
    odometry_msg.header.stamp = now;
    odometry_msg.child_frame_id = child_frame;

    odometry_msg.pose.pose.position.x = 0.0f;
    odometry_msg.pose.pose.position.y = 0.0f;
    odometry_msg.pose.pose.position.z = 0.0f;
    odometry_msg.pose.pose.orientation.x = id.getX();
    odometry_msg.pose.pose.orientation.y = id.getY();
    odometry_msg.pose.pose.orientation.z = id.getZ();
    odometry_msg.pose.pose.orientation.w = id.getW();
    odometry_msg.pose.covariance = std::array<double, 36>{
      {0.75, 0.04, 0.1, 0, 0, 0,
        0.04, 0.7, 0.4, 0, 0, 0,
        0.1, 0.4, 0.5, 0, 0, 0,
        0, 0, 0, 0.8, 0.25, 0.06,
        0, 0, 0, 0.25, 0.3, 0.22,
        0, 0, 0, 0.06, 0.22, 0.6}
    };


    odometry_msg.twist.twist.linear.x = 0.3f;

    publisher->publish(odometry_msg);
  }

  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster;
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher;
};


}  // namespace nodes

#endif  // RVIZ_DEFAULT_PLUGINS__PUBLISHERS__ODOMETRY_PUBLISHER_HPP_
