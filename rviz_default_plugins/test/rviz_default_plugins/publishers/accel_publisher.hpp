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

#ifndef RVIZ_DEFAULT_PLUGINS__PUBLISHERS__ACCEL_PUBLISHER_HPP_
#define RVIZ_DEFAULT_PLUGINS__PUBLISHERS__ACCEL_PUBLISHER_HPP_

#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/accel_stamped.hpp"
#include "std_msgs/msg/header.hpp"

using namespace std::chrono_literals;  // NOLINT

namespace nodes
{

class AccelPublisher : public rclcpp::Node
{
public:
  AccelPublisher();

private:
  geometry_msgs::msg::AccelStamped createAccelMessage();

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::AccelStamped>::SharedPtr publisher_;
};

AccelPublisher::AccelPublisher()
: Node("accel_publisher")
{
  publisher_ = this->create_publisher<geometry_msgs::msg::AccelStamped>("accel", 10);

  auto timer_callback =
    [this]() -> void {
      auto message = createAccelMessage();
      this->publisher_->publish(message);
    };
  timer_ = this->create_wall_timer(500ms, timer_callback);
}

geometry_msgs::msg::AccelStamped AccelPublisher::createAccelMessage()
{
  geometry_msgs::msg::AccelStamped accelMessage;

  accelMessage.header = std_msgs::msg::Header();
  accelMessage.header.frame_id = "accel_frame";
  accelMessage.header.stamp = rclcpp::Clock().now();

  accelMessage.accel.linear.x = 1;
  accelMessage.accel.linear.y = 3;
  accelMessage.accel.linear.z = 2;

  accelMessage.accel.angular.x = 3;
  accelMessage.accel.angular.y = 2;
  accelMessage.accel.angular.z = 1;

  return accelMessage;
}

}  // namespace nodes

#endif  // RVIZ_DEFAULT_PLUGINS__PUBLISHERS__ACCEL_PUBLISHER_HPP_
