// Copyright 2025 Open Source Robotics Foundation, Inc.
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

#ifndef RVIZ_DEFAULT_PLUGINS__ROS_RESOURCE_RETRIEVER_HPP_
#define RVIZ_DEFAULT_PLUGINS__ROS_RESOURCE_RETRIEVER_HPP_

#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>

#include <rclcpp/callback_group.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <resource_retriever/plugins/retriever_plugin.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_resource_interfaces/srv/get_resource.hpp>

/// Plugin for resource_retriever that loads resources from a ROS interface.
class RosResourceRetriever : public resource_retriever::plugins::RetrieverPlugin
{
  using GetResource = rviz_resource_interfaces::srv::GetResource;

  RosResourceRetriever() = delete;

  static constexpr std::string_view service_name = "/rviz/get_resource";

public:
  explicit RosResourceRetriever(
    rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr weak_ros_iface);

  ~RosResourceRetriever() override = default;

  std::string name() override;

  bool can_handle(const std::string & url) override;

  resource_retriever::ResourceSharedPtr get_shared(const std::string & url) override;

private:
  // It should be safe to keep a shared pointer to the node here, because this
  // plugin will be destroyed with the resource retriever in the marker display,
  // which should be destroyed along before the node abstraction is destroyed.
  // Also, since we're keeping callback groups and clients around, we need to
  // ensure the node stays around too.
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Client<GetResource>::SharedPtr client_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Logger logger_;

  // Map of the resource path to a pair with the etag value and the memory resource that is cached.
  std::unordered_map<
    std::string,
    std::pair<std::string, resource_retriever::ResourceSharedPtr>
  > cached_resources_;
};

#endif  // RVIZ_DEFAULT_PLUGINS__ROS_RESOURCE_RETRIEVER_HPP_
