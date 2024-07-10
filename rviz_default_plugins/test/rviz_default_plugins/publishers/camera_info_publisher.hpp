// Copyright (c) 2018, Bosch Software Innovations GmbH.
// All rights reserved.
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#ifndef RVIZ_DEFAULT_PLUGINS__PUBLISHERS__CAMERA_INFO_PUBLISHER_HPP_
#define RVIZ_DEFAULT_PLUGINS__PUBLISHERS__CAMERA_INFO_PUBLISHER_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "std_msgs/msg/header.hpp"

using namespace std::chrono_literals;  // NOLINT

namespace nodes
{

class CameraInfoPublisher : public rclcpp::Node
{
public:
  explicit CameraInfoPublisher(std::string frame_id = "camera_info_frame")
  : Node("camera_info_publisher")
  {
    publisher = this->create_publisher<sensor_msgs::msg::CameraInfo>("/image/camera_info", 10);
    timer = this->create_wall_timer(500ms, std::bind(&CameraInfoPublisher::timer_callback, this));
    this->frame_id = frame_id;
  }

private:
  void timer_callback()
  {
    auto message = sensor_msgs::msg::CameraInfo();
    message.header = std_msgs::msg::Header();
    message.header.frame_id = this->frame_id;
    message.header.stamp = rclcpp::Clock().now();

    message.width = 320;
    message.height = 240;

    message.p = {{160.0f, 0.0f, 160.0f, 0.0f, 0.0f, 120.0f, 120.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f}};

    publisher->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr publisher;
  std::string frame_id;
};

}  // namespace nodes

#endif  // RVIZ_DEFAULT_PLUGINS__PUBLISHERS__CAMERA_INFO_PUBLISHER_HPP_
