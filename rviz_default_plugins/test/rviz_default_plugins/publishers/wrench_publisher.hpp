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

#ifndef RVIZ_DEFAULT_PLUGINS__PUBLISHERS__WRENCH_PUBLISHER_HPP_
#define RVIZ_DEFAULT_PLUGINS__PUBLISHERS__WRENCH_PUBLISHER_HPP_

#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "std_msgs/msg/header.hpp"

using namespace std::chrono_literals; //NOLINT

namespace nodes
{

class WrenchPublisher : public rclcpp::Node
{
public:
  WrenchPublisher();

private:
  geometry_msgs::msg::WrenchStamped createWrenchMessage();

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr publisher_;
};

WrenchPublisher::WrenchPublisher()
: Node("wrench_publisher")
{
  publisher_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("wrench", 10);

  auto timer_callback =
    [this]() -> void {
      auto message = createWrenchMessage();
      this->publisher_->publish(message);
    };
  timer_ = this->create_wall_timer(500ms, timer_callback);
}

geometry_msgs::msg::WrenchStamped WrenchPublisher::createWrenchMessage()
{
  geometry_msgs::msg::WrenchStamped wrenchMessage;

  wrenchMessage.header = std_msgs::msg::Header();
  wrenchMessage.header.frame_id = "wrench_frame";
  wrenchMessage.header.stamp = rclcpp::Clock().now();

  wrenchMessage.wrench.force.x = 1;
  wrenchMessage.wrench.force.y = 3;
  wrenchMessage.wrench.force.z = 2;

  wrenchMessage.wrench.torque.x = 3;
  wrenchMessage.wrench.torque.y = 2;
  wrenchMessage.wrench.torque.z = 1;

  return wrenchMessage;
}

}  // namespace nodes

#endif  // RVIZ_DEFAULT_PLUGINS__PUBLISHERS__WRENCH_PUBLISHER_HPP_
