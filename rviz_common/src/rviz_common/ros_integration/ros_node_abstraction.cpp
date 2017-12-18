/*
 * Copyright (c) 2017, Open Source Robotics Foundation, Inc.
 * Copyright (c) 2017, Bosch Software Innovations GmbH.
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

#include "rviz_common/ros_integration/ros_node_abstraction.hpp"

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "ros_node_storage_iface.hpp"
#include "ros_node_storage.hpp"

namespace rviz_common
{
namespace ros_integration
{

RosNodeAbstraction::RosNodeAbstraction(const std::string & node_name)
: RosNodeAbstraction(node_name, std::make_shared<RosNodeStorage>())
{}

RosNodeAbstraction::RosNodeAbstraction(
  const std::string & node_name, std::shared_ptr<RosNodeStorageIface> ros_node_storage)
: node_name_(node_name), ros_node_storage_(ros_node_storage)
{
  if (!ros_node_storage_->has_rclcpp_node_by_name(node_name_)) {
    ros_node_storage_->store_rclcpp_node_by_name(
      node_name_, std::make_shared<rclcpp::Node>(node_name_));
  }
}

std::string
RosNodeAbstraction::get_node_name()
{
  return node_name_;
}

std::map<std::string, std::vector<std::string>>
RosNodeAbstraction::get_topic_names_and_types()
{
  rclcpp::Node::SharedPtr node = ros_node_storage_->get_rclcpp_node_by_name(node_name_);
  if (node == nullptr) {
    throw std::runtime_error("given node name '" + node_name_ + "' not found");
  }
  return node->get_topic_names_and_types();
}

}  // namespace ros_integration
}  // namespace rviz_common
