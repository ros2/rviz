/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * Copyright (c) 2017, Open Source Robotics Foundation, Inc.
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

#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "geometry_msgs/msg/point32.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  int rate = 1;
  bool moving = true;
  int size = 100;

  if (argc > 1) {
    rate = atoi(argv[1]);
  }
  if (argc > 2) {
    moving = (atoi(argv[2]) != 0);
  }
  if (argc > 3) {
    size = atoi(argv[3]);
  }

  auto node = rclcpp::Node::make_shared("send_lots_of_points");

  auto pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", 100);
  rclcpp::Rate loop_rate(rate);

  sensor_msgs::msg::PointCloud2 msg;
  int width = size;
  int length = 2 * size;
  msg.header.frame_id = "world";

  // Fill 2 using an iterator
  sensor_msgs::PointCloud2Modifier modifier(msg);
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  int count = 0;
  printf("cloud_msg_.point_step %d\n", msg.point_step);
  while (rclcpp::ok() ) {
    width++;
    modifier.resize(width, length + (count % 2));
    sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(msg, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(msg, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(msg, "b");
    for (int x = 0; x < width; x++) {
      for (int y = 0; y < length; y++) {
        if (count % 2) {
          *iter_x = -.1f;
          *iter_y = -.1f;
          *iter_z = 1.1f;
        } else {
          *iter_x = static_cast<float>(x / 100.0);
          *iter_y = static_cast<float>(y / 100.0);
          *iter_z = ((x + y + count) % 100) / 100.0f;
        }

        *iter_r = 255;
        *iter_g = 0;
        *iter_b = 0;

        ++iter_x;
        ++iter_y;
        ++iter_z;
        ++iter_r;
        ++iter_g;
        ++iter_b;
      }
    }

    msg.header.stamp = node->now();

    printf(
      "publishing at %d hz, %s, %d x %d points.\n",
      rate, (moving ? "moving" : "static"), width, length);

    pub->publish(msg);

    rclcpp::spin_some(node);
    loop_rate.sleep();
    if (moving) {
      ++count;
    }
  }
}
