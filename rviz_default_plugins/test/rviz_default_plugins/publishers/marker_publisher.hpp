/*
 * Copyright (c) 2018, Bosch Software Innovations GmbH.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef RVIZ_DEFAULT_PLUGINS__PUBLISHERS__MARKER_PUBLISHER_HPP_
#define RVIZ_DEFAULT_PLUGINS__PUBLISHERS__MARKER_PUBLISHER_HPP_

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"

// TODO(greimela): Workaround for duplicate constant definition
// compare https://github.com/ros2/common_interfaces/issues/44
#if defined(_WIN32) && defined(DELETE)
# undef DELETE
#endif
#include "visualization_msgs/msg/marker.hpp"

using namespace std::chrono_literals;  // NOLINT

namespace nodes
{

class MarkerPublisher : public rclcpp::Node
{
public:
  MarkerPublisher()
  : Node("marker_publisher")
  {
    publisher = this->create_publisher<visualization_msgs::msg::Marker>("marker");
    timer = this->create_wall_timer(200ms, std::bind(&MarkerPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto header = std_msgs::msg::Header();
    header.frame_id = "marker_frame";
    header.stamp = rclcpp::Clock().now();

    std::string ns = "test_ns";
    int id = 0;

    auto sphere_marker = createSphereMarker(header, ns, ++id);
    publisher->publish(sphere_marker);

    auto line_strip_marker = createLineStripMarker(header, ns, ++id);
    publisher->publish(line_strip_marker);

    auto cube_list_marker = createCubeListMarker(header, ns, ++id);
    publisher->publish(cube_list_marker);
  }

  visualization_msgs::msg::Marker createSphereMarker(
    std_msgs::msg::Header header, const std::string & ns, int id)
  {
    auto marker = visualization_msgs::msg::Marker();
    marker.header = header;
    marker.ns = ns;
    marker.id = id;

    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.scale.x = 6;
    marker.scale.y = 6;
    marker.scale.z = 6;

    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = -2;

    return marker;
  }

  visualization_msgs::msg::Marker createLineStripMarker(
    std_msgs::msg::Header header, const std::string & ns, int id)
  {
    auto marker = visualization_msgs::msg::Marker();
    marker.header = header;
    marker.ns = ns;
    marker.id = id;

    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.scale.x = 3;

    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    auto p1 = geometry_msgs::msg::Point();
    p1.x = 3;
    p1.y = 3;
    p1.z = 0;

    auto p2 = geometry_msgs::msg::Point();
    p2.x = 3;
    p2.y = -3;
    p2.z = 0;

    auto p3 = geometry_msgs::msg::Point();
    p3.x = -3;
    p3.y = -3;
    p3.z = 0;

    auto p4 = geometry_msgs::msg::Point();
    p4.x = -3;
    p4.y = 3;
    p4.z = 0;

    marker.points = std::vector<geometry_msgs::msg::Point>();
    marker.points.push_back(p1);
    marker.points.push_back(p2);
    marker.points.push_back(p3);
    marker.points.push_back(p4);

    return marker;
  }

  visualization_msgs::msg::Marker createCubeListMarker(
    std_msgs::msg::Header header, const std::string & ns, int id)
  {
    auto marker = visualization_msgs::msg::Marker();
    marker.header = header;
    marker.ns = ns;
    marker.id = id;

    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.scale.x = 3;
    marker.scale.y = 3;
    marker.scale.z = 3;

    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    auto p1 = geometry_msgs::msg::Point();
    p1.x = 3;
    p1.y = 3;
    p1.z = 0;

    auto p2 = geometry_msgs::msg::Point();
    p2.x = -3;
    p2.y = -3;
    p2.z = 0;

    auto p3 = geometry_msgs::msg::Point();
    p3.x = -3;
    p3.y = 3;
    p3.z = 0;

    auto p4 = geometry_msgs::msg::Point();
    p4.x = 3;
    p4.y = -3;
    p4.z = 0;

    marker.points = std::vector<geometry_msgs::msg::Point>();
    marker.points.push_back(p1);
    marker.points.push_back(p2);
    marker.points.push_back(p3);
    marker.points.push_back(p4);

    return marker;
  }

  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher;
};

}  // namespace nodes

#endif  // RVIZ_DEFAULT_PLUGINS__PUBLISHERS__MARKER_PUBLISHER_HPP_
