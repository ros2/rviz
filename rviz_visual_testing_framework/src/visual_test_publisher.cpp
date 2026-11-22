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

#include "rviz_visual_testing_framework/visual_test_publisher.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/node.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.hpp"

#include "rviz_visual_testing_framework/internal/transform_message_creator.hpp"

VisualTestPublisher::VisualTestPublisher(
  std::shared_ptr<rclcpp::Node> publisher_node, std::string frame_name)
{
  nodes_spinning_ = true;
  std::vector<PublisherWithFrame> publishers = {PublisherWithFrame(publisher_node, frame_name)};
  publisher_thread_ = std::thread(
    &VisualTestPublisher::publishOnFrame, this, publishers);
}

VisualTestPublisher::VisualTestPublisher(std::vector<PublisherWithFrame> publishers)
{
  nodes_spinning_ = true;
  publisher_thread_ = std::thread(&VisualTestPublisher::publishOnFrame, this, publishers);
}

VisualTestPublisher::~VisualTestPublisher()
{
  nodes_spinning_ = false;
  publisher_thread_.join();
}

void VisualTestPublisher::publishOnFrame(std::vector<PublisherWithFrame> publishers)
{
  auto transformer_publisher_node = std::make_shared<rclcpp::Node>("static_transform_publisher");
  tf2_ros::StaticTransformBroadcaster broadcaster(*transformer_publisher_node);

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
