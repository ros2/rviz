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

#include "marker_messages.hpp"

#include <memory>
#include <string>

#include <OgreVector.h>

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/header.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace testing
{

geometry_msgs::msg::Point create_point(float x, float y, float z)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

std_msgs::msg::ColorRGBA color(float r, float g, float b, float a)
{
  std_msgs::msg::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

visualization_msgs::msg::Marker createDefaultMessage(int32_t type)
{
  auto header = std_msgs::msg::Header();
  header.frame_id = "marker_frame";
  header.stamp = rclcpp::Clock().now();

  std::string ns = "test_ns";
  int id = 0;

  auto marker = visualization_msgs::msg::Marker();
  marker.header = header;
  marker.ns = ns;
  marker.id = id;

  marker.type = type;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.scale.x = 1;
  marker.scale.y = 0.2f;
  marker.scale.z = 0.2f;

  marker.color.a = 1.0f;
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 1.0f;

  marker.pose.position.x = cos(Ogre::Math::PI / 2);
  marker.pose.position.y = sin(Ogre::Math::PI / 2);
  marker.pose.position.z = 5.0f;

  marker.pose.orientation.x = 1.0f;
  marker.pose.orientation.y = 1.0f;
  marker.pose.orientation.z = 1.0f;
  marker.pose.orientation.w = 1.0f;

  marker.text = "Displaytext";

  marker.mesh_resource = "package://rviz_default_plugins/test_meshes/pr2-base.dae";
  marker.mesh_use_embedded_materials = true;
  return marker;
}

visualization_msgs::msg::Marker createMessageWithPoints(int32_t type)
{
  auto marker = createDefaultMessage(type);
  marker.points.push_back(create_point(2, 0, 0));
  marker.points.push_back(create_point(1, 1, 0));
  return marker;
}

visualization_msgs::msg::Marker createMessageWithColorPerPoint(int32_t type)
{
  auto marker = createMessageWithPoints(type);
  marker.colors.push_back(color(1.0f, 0.0f, 0.5f, 0.5f));
  marker.colors.push_back(color(0.5f, 0.6f, 0.0f, 0.3f));
  return marker;
}

}  // namespace testing
