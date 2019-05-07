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

#ifndef RVIZ_DEFAULT_PLUGINS__PUBLISHERS__MARKER_ARRAY_PUBLISHER_HPP_
#define RVIZ_DEFAULT_PLUGINS__PUBLISHERS__MARKER_ARRAY_PUBLISHER_HPP_

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"

// TODO(greimela): Workaround for duplicate constant definition
// compare https://github.com/ros2/common_interfaces/issues/44
#if defined(_WIN32) && defined(DELETE)
# undef DELETE
#endif
#include "visualization_msgs/msg/marker_array.hpp"
#include "marker_publisher.hpp"

using namespace std::chrono_literals;  // NOLINT

namespace nodes
{

class MarkerArrayPublisher : public MarkerPublisher
{
public:
  MarkerArrayPublisher()
  : MarkerPublisher("marker_array_publisher")
  {
    array_publisher_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>("marker_array", 10);
  }

private:
  void timer_callback() override
  {
    auto header = std_msgs::msg::Header();
    header.frame_id = "marker_array_frame";
    header.stamp = rclcpp::Clock().now();

    std::string ns = "test_ns";
    int id = 0;

    auto first_sphere_marker = createSphereMarker(header, ns, ++id);
    auto second_sphere_marker = createSphereMarker(header, ns, ++id);
    auto cube_list_marker = createCubeListMarker(header, ns, ++id);
    cube_list_marker.color.g = 0;
    first_sphere_marker.color.g = 0;
    first_sphere_marker.color.b = 1;
    second_sphere_marker.color.g = 0;
    second_sphere_marker.color.b = 1;
    first_sphere_marker.pose.position.x = -2;
    second_sphere_marker.pose.position.x = 2;

    auto marker_array = visualization_msgs::msg::MarkerArray();
    marker_array.markers = std::vector<visualization_msgs::msg::Marker>();
    marker_array.markers.emplace_back(first_sphere_marker);
    marker_array.markers.emplace_back(second_sphere_marker);
    marker_array.markers.emplace_back(createLineStripMarker(header, ns, ++id));
    marker_array.markers.emplace_back(cube_list_marker);

    array_publisher_->publish(marker_array);
  }

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr array_publisher_;
};

}  // namespace nodes

#endif  // RVIZ_DEFAULT_PLUGINS__PUBLISHERS__MARKER_ARRAY_PUBLISHER_HPP_
