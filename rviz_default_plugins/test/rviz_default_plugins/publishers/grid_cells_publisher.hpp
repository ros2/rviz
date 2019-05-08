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

#ifndef RVIZ_DEFAULT_PLUGINS__PUBLISHERS__GRID_CELLS_PUBLISHER_HPP_
#define RVIZ_DEFAULT_PLUGINS__PUBLISHERS__GRID_CELLS_PUBLISHER_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "std_msgs/msg/header.hpp"
#include "nav_msgs/msg/grid_cells.hpp"
#include "geometry_msgs/msg/point.hpp"

using namespace std::chrono_literals;  // NOLINT

namespace nodes
{

class GridCellsPublisher : public rclcpp::Node
{
public:
  GridCellsPublisher()
  : Node("grid_cells_publisher")
  {
    publisher = this->create_publisher<nav_msgs::msg::GridCells>("grid_cells", 10);
    timer = this->create_wall_timer(500ms, std::bind(&GridCellsPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = nav_msgs::msg::GridCells();
    message.header = std_msgs::msg::Header();
    message.header.frame_id = "grid_cells_frame";
    message.header.stamp = rclcpp::Clock().now();

    message.cell_width = 6;
    message.cell_height = 5;

    std::vector<geometry_msgs::msg::Point> points;
    points.push_back(point(0, -1, 0));
    points.push_back(point(3, 1, 0));
    points.push_back(point(-3, 1, 0));
    message.cells = points;

    publisher->publish(message);
  }

  geometry_msgs::msg::Point point(float x, float y, float z)
  {
    auto point = geometry_msgs::msg::Point();
    point.x = x;
    point.y = y;
    point.z = z;
    return point;
  }

  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<nav_msgs::msg::GridCells>::SharedPtr publisher;
};

}  // namespace nodes

#endif  // RVIZ_DEFAULT_PLUGINS__PUBLISHERS__GRID_CELLS_PUBLISHER_HPP_
