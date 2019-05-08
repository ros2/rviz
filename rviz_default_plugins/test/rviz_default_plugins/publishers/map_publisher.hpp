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

#ifndef RVIZ_DEFAULT_PLUGINS__PUBLISHERS__MAP_PUBLISHER_HPP_
#define RVIZ_DEFAULT_PLUGINS__PUBLISHERS__MAP_PUBLISHER_HPP_

#include <cmath>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

using namespace std::chrono_literals;  // NOLINT

namespace nodes
{

class MapPublisher : public rclcpp::Node
{
public:
  MapPublisher()
  : Node("map_publisher")
  {
    publisher = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);
    timer = this->create_wall_timer(500ms, std::bind(&MapPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto header = std_msgs::msg::Header();
    header.frame_id = "map_frame";
    header.stamp = rclcpp::Clock().now();

    auto meta_data = nav_msgs::msg::MapMetaData();
    meta_data.height = 256;
    meta_data.width = 256;
    meta_data.map_load_time = rclcpp::Clock().now();
    meta_data.resolution = 1;

    meta_data.origin.position.x = -128;
    meta_data.origin.position.y = -128;
    meta_data.origin.position.z = 0;

    meta_data.origin.orientation.x = 0;
    meta_data.origin.orientation.y = 0;
    meta_data.origin.orientation.z = 0;
    meta_data.origin.orientation.w = 1;

    auto new_data = std::vector<int8_t>();
    for (uint32_t i = 0; i < meta_data.width; i++) {
      for (uint32_t j = 0; j < meta_data.height; j++) {
        new_data.emplace_back(i);
      }
    }

    nav_msgs::msg::OccupancyGrid occupancy_grid;
    occupancy_grid.header = header;
    occupancy_grid.info = meta_data;
    occupancy_grid.data = new_data;

    publisher->publish(occupancy_grid);
  }

  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher;
};

}  // namespace nodes

#endif  // RVIZ_DEFAULT_PLUGINS__PUBLISHERS__MAP_PUBLISHER_HPP_
