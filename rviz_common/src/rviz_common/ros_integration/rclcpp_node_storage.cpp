/*
 * Copyright (c) 2017, Open Source Robotics Foundation, Inc.
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

#include "./rclcpp_node_storage.hpp"

#include <map>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace rviz_common
{
namespace ros_integration
{

static std::map<std::string, rclcpp::Node::SharedPtr> __nodes_by_name;
static std::mutex __nodes_by_name_mutex;

void
store_rclcpp_node_by_name(
  const std::string & node_name,
  const std::shared_ptr<rclcpp::Node> & node)
{
  std::lock_guard<std::mutex> lock(__nodes_by_name_mutex);
  __nodes_by_name[node_name] = node;
}

bool
__has_rclcpp_node_by_name(const std::string & node_name)
{
  return __nodes_by_name.count(node_name) != 0;
}

std::shared_ptr<rclcpp::Node>
get_rclcpp_node_by_name(const std::string & node_name)
{
  std::lock_guard<std::mutex> lock(__nodes_by_name_mutex);
  if (!__has_rclcpp_node_by_name(node_name)) {
    return nullptr;
  }
  return __nodes_by_name[node_name];
}

bool
has_rclcpp_node_by_name(const std::string & node_name)
{
  std::lock_guard<std::mutex> lock(__nodes_by_name_mutex);
  return __has_rclcpp_node_by_name(node_name);
}

}  // namespace ros_integration
}  // namespace rviz_common
