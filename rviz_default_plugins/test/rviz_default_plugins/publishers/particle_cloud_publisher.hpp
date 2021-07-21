/*
 * Copyright (c) 2018, Bosch Software Innovations GmbH.
 * Copyright (c) 2020, Sarthak Mittal.
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

#ifndef RVIZ_DEFAULT_PLUGINS__PUBLISHERS__PARTICLE_CLOUD_PUBLISHER_HPP_
#define RVIZ_DEFAULT_PLUGINS__PUBLISHERS__PARTICLE_CLOUD_PUBLISHER_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "std_msgs/msg/header.hpp"
#include "nav2_msgs/msg/particle_cloud.hpp"

using namespace std::chrono_literals;  // NOLINT

namespace nodes
{
class ParticleCloudPublisher : public rclcpp::Node
{
public:
  ParticleCloudPublisher()
  : Node("particle_cloud_publisher")
  {
    publisher = this->create_publisher<nav2_msgs::msg::ParticleCloud>("particle_cloud", 10);
    timer = this->create_wall_timer(
      500ms, std::bind(&ParticleCloudPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = nav2_msgs::msg::ParticleCloud();
    message.header = std_msgs::msg::Header();
    message.header.frame_id = "particle_cloud_frame";
    message.header.stamp = rclcpp::Clock().now();

    for (int i = 0; i < 3; ++i) {
      nav2_msgs::msg::Particle particle;

      particle.pose.position.x = 0;
      particle.pose.position.y = i - 1;
      particle.pose.position.z = 0;

      particle.pose.orientation.x = 0;
      particle.pose.orientation.y = 0;
      particle.pose.orientation.z = 0;
      particle.pose.orientation.w = 1;

      particle.weight = 1;

      message.particles.push_back(particle);
    }

    publisher->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<nav2_msgs::msg::ParticleCloud>::SharedPtr publisher;
};

}  // namespace nodes

#endif  // RVIZ_DEFAULT_PLUGINS__PUBLISHERS__PARTICLE_CLOUD_PUBLISHER_HPP_
