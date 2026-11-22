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

#include "rviz_visual_testing_framework/transform_publisher.hpp"

#include <memory>
#include <thread>
#include <vector>

#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rate.hpp"
#include "tf2_ros/static_transform_broadcaster.hpp"

#include "rviz_visual_testing_framework/internal/transform_message_creator.hpp"

geometry_msgs::msg::TransformStamped StaticTransform::createStaticTransformMessage()
{
  return createStaticTransformMessageFor(
    origin_frame, destination_frame, x, y, z, roll, pitch, yaw);
}

TransformPublisher::TransformPublisher(std::vector<StaticTransform> transforms)
{
  nodes_spinning_ = true;
  transforms_ = transforms;
  publisher_thread_ = std::thread(
    &TransformPublisher::publishOnFrame, this);
}

TransformPublisher::~TransformPublisher()
{
  nodes_spinning_ = false;
  publisher_thread_.join();
}

void TransformPublisher::publishOnFrame()
{
  auto transformer_publisher_node = std::make_shared<rclcpp::Node>("static_transform_publisher");
  tf2_ros::StaticTransformBroadcaster broadcaster(*transformer_publisher_node);

  rclcpp::WallRate loop_rate(0.2);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(transformer_publisher_node);

  while (nodes_spinning_) {
    for (auto transform : transforms_) {
      broadcaster.sendTransform(transform.createStaticTransformMessage());
    }
    executor.spin_some();
    loop_rate.sleep();
  }
}
