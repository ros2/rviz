// Copyright (c) 2026, Open Source Robotics Foundation, Inc.
// All rights reserved.
//
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


#ifndef RVIZ_DEFAULT_PLUGINS__PUBLISHERS__COMPRESSED_IMAGE_PUBLISHER_HPP_
#define RVIZ_DEFAULT_PLUGINS__PUBLISHERS__COMPRESSED_IMAGE_PUBLISHER_HPP_

#include <chrono>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "std_msgs/msg/header.hpp"

using namespace std::chrono_literals;  // NOLINT

namespace nodes
{

class CompressedImagePublisher : public rclcpp::Node
{
public:
  explicit CompressedImagePublisher(const std::string & topic_name = "image")
  : Node("compressed_image_publisher")
  {
    publisher = this->create_publisher<sensor_msgs::msg::CompressedImage>(topic_name, 10);
    timer = this->create_wall_timer(
      100ms, std::bind(&CompressedImagePublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = sensor_msgs::msg::CompressedImage();
    message.header = std_msgs::msg::Header();
    message.header.frame_id = "image_frame";
    message.header.stamp = rclcpp::Clock().now();

    cv::Mat image(200, 300, CV_8UC3, cv::Scalar(0, 255, 0));
    std::vector<uchar> compressed_image;
    cv::imencode(".jpg", image, compressed_image);

    message.data.assign(compressed_image.begin(), compressed_image.end());
    message.format = "jpeg";
    publisher->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher;
};

}  // namespace nodes

#endif  // RVIZ_DEFAULT_PLUGINS__PUBLISHERS__COMPRESSED_IMAGE_PUBLISHER_HPP_
