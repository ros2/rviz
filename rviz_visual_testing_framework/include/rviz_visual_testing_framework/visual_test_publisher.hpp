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

#ifndef RVIZ_VISUAL_TESTING_FRAMEWORK__VISUAL_TEST_PUBLISHER_HPP_
#define RVIZ_VISUAL_TESTING_FRAMEWORK__VISUAL_TEST_PUBLISHER_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "internal/transform_message_creator.hpp"

struct PublisherWithFrame
{
  PublisherWithFrame(std::shared_ptr<rclcpp::Node> publisher_node, std::string frame_name)
  : publisher_node_(publisher_node), frame_name_(frame_name) {}

  std::shared_ptr<rclcpp::Node> publisher_node_;
  std::string frame_name_;
};

/**
 * This class is used internally to set up publishers and automatically publish simple static
 * transformations. You can use this class in your test with the frame_name of your publisher to
 * make sure that tf2 transformations are valid.
 */
class VisualTestPublisher
{
public:
  VisualTestPublisher(std::shared_ptr<rclcpp::Node> publisher_node, std::string frame_name)
  {
    nodes_spinning_ = true;
    std::vector<PublisherWithFrame> publishers = {PublisherWithFrame(publisher_node, frame_name)};
    publisher_thread_ = std::thread(
      &VisualTestPublisher::publishOnFrame, this, publishers);
  }

  explicit VisualTestPublisher(std::vector<PublisherWithFrame> publishers)
  {
    nodes_spinning_ = true;
    publisher_thread_ = std::thread(&VisualTestPublisher::publishOnFrame, this, publishers);
  }

  ~VisualTestPublisher()
  {
    nodes_spinning_ = false;
    publisher_thread_.join();
  }

private:
  void publishOnFrame(std::vector<PublisherWithFrame> publishers)
  {
    auto transformer_publisher_node = std::make_shared<rclcpp::Node>("static_transform_publisher");
    tf2_ros::StaticTransformBroadcaster broadcaster(transformer_publisher_node);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(transformer_publisher_node);

    std::vector<geometry_msgs::msg::TransformStamped> transform_messages;
    for (auto publisherWithFrame : publishers) {
      executor.add_node(publisherWithFrame.publisher_node_);
      transform_messages.push_back(
        createStaticTransformMessageFor("map", publisherWithFrame.frame_name_));
    }
    while (nodes_spinning_) {
      for (const auto & msg : transform_messages) {
        broadcaster.sendTransform(msg);
      }
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  }

  std::atomic<bool> nodes_spinning_;
  std::thread publisher_thread_;
};

#endif  // RVIZ_VISUAL_TESTING_FRAMEWORK__VISUAL_TEST_PUBLISHER_HPP_
