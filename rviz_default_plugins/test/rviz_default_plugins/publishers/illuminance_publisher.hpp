/*
 * Copyright (c) 2017, Bosch Software Innovations GmbH.
 * Copyright (c) 2018, TNG Technology Consulting GmbH.
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

#ifndef RVIZ_DEFAULT_PLUGINS__PUBLISHERS__ILLUMINANCE_PUBLISHER_HPP_
#define RVIZ_DEFAULT_PLUGINS__PUBLISHERS__ILLUMINANCE_PUBLISHER_HPP_

#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/illuminance.hpp"
#include "std_msgs/msg/header.hpp"

using namespace std::chrono_literals; //NOLINT

namespace nodes
{

class IlluminancePublisher : public rclcpp::Node
{
public:
  IlluminancePublisher();
  void setIlluminance(double illuminance);

private:
  sensor_msgs::msg::Illuminance createIlluminanceMessage();

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Illuminance>::SharedPtr publisher_;

  double illuminance_;
};

IlluminancePublisher::IlluminancePublisher()
: Node("illuminance_publisher"), illuminance_(0.)
{
  publisher_ = this->create_publisher<sensor_msgs::msg::Illuminance>("illuminance", 10);

  auto timer_callback =
    [this]() -> void {
      auto message = createIlluminanceMessage();
      this->publisher_->publish(message);
    };
  timer_ = this->create_wall_timer(500ms, timer_callback);
}

sensor_msgs::msg::Illuminance IlluminancePublisher::createIlluminanceMessage()
{
  sensor_msgs::msg::Illuminance illuminanceMessage;

  illuminanceMessage.header = std_msgs::msg::Header();
  illuminanceMessage.header.frame_id = "illuminance_frame";
  illuminanceMessage.header.stamp = rclcpp::Clock().now();

  illuminanceMessage.illuminance = illuminance_;
  illuminanceMessage.variance = 1.0;

  return illuminanceMessage;
}

void IlluminancePublisher::setIlluminance(double illuminance)
{
  illuminance_ = illuminance;
}

}  // namespace nodes

#endif  // RVIZ_DEFAULT_PLUGINS__PUBLISHERS__ILLUMINANCE_PUBLISHER_HPP_
