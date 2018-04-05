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

#ifndef RVIZ_VISUAL_TESTING_FRAMEWORK__TESTS__EXAMPLE_NODES_HPP_
#define RVIZ_VISUAL_TESTING_FRAMEWORK__TESTS__EXAMPLE_NODES_HPP_

#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"

using namespace std::chrono_literals;  // NOLINT

namespace nodes
{

class PointCloudPublisher : public rclcpp::Node
{
public:
  explicit PointCloudPublisher(std::vector<geometry_msgs::msg::Point32> points)
  : Node("pointcloud_publisher"),
    points_(points)
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud>("pointcloud");
    timer_ = this->create_wall_timer(500ms, std::bind(&PointCloudPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = sensor_msgs::msg::PointCloud();
    message.header = std_msgs::msg::Header();
    message.header.frame_id = "pointcloud_frame";
    message.header.stamp = rclcpp::Clock().now();
    message.points = points_;

    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr publisher_;
  std::vector<geometry_msgs::msg::Point32> points_;
};

geometry_msgs::msg::TransformStamped createStaticTransformMessage(
  std::string header_frame_id, std::string child_frame_id)
{
  geometry_msgs::msg::TransformStamped message;

  message.transform.translation.x = 0;
  message.transform.translation.y = 0;
  message.transform.translation.z = 0;

  tf2::Quaternion quat;
  quat.setRPY(0, 0, 0);
  message.transform.rotation.x = quat.x();
  message.transform.rotation.y = quat.y();
  message.transform.rotation.z = quat.z();
  message.transform.rotation.w = quat.w();

  message.header.stamp = rclcpp::Clock().now();
  message.header.frame_id = header_frame_id;
  message.child_frame_id = child_frame_id;

  return message;
}

}  // namespace nodes

#endif  // RVIZ_VISUAL_TESTING_FRAMEWORK__TESTS__EXAMPLE_NODES_HPP_
