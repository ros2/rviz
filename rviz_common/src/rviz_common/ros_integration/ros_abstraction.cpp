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

#include "rviz_common/ros_integration/ros_abstraction.hpp"

#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace rviz_common
{
namespace ros_integration
{

std::string
RosAbstraction::init(int argc, char ** argv, const std::string & name, bool anonymous_name)
{
  std::string final_name = name;
  if (anonymous_name) {
    // TODO(wjwwood): add anonymous name feature to rclcpp or somehow make name
    //                anonymouse here.
    throw std::runtime_error("'anonymous_name' feature not implemented");
    // final_name = <the full anonymous node name>;
  }
  // TODO(wjwwood): this will throw on repeated calls, maybe avoid that?
  rclcpp::init(argc, argv);
  if (has_rclcpp_node_by_name(name)) {
    // TODO(wjwwood): make a better exception type rather than using std::runtime_error.
    throw std::runtime_error("Node with name " + final_name + " already exists.");
  }
  store_rclcpp_node_by_name(name, std::make_shared<rclcpp::Node>(final_name));
  return final_name;
}

bool
RosAbstraction::ok(const std::string & node_name)
{
  return rclcpp::ok() && has_rclcpp_node_by_name(node_name);
}

void
RosAbstraction::shutdown()
{
  clear_rclcpp_nodes();
  rclcpp::shutdown();
}

void
RosAbstraction::store_rclcpp_node_by_name(
  const std::string & node_name,
  const std::shared_ptr<rclcpp::Node> & node)
{
  std::lock_guard<std::mutex> lock(nodes_by_name_mutex_);
  nodes_by_name_[node_name] = node;
}

std::shared_ptr<rclcpp::Node>
RosAbstraction::get_rclcpp_node_by_name(const std::string & node_name)
{
  std::lock_guard<std::mutex> lock(nodes_by_name_mutex_);
  if (nodes_by_name_.count(node_name) == 0) {
    return nullptr;
  }
  return nodes_by_name_[node_name];
}

bool
RosAbstraction::has_rclcpp_node_by_name(const std::string & node_name)
{
  std::lock_guard<std::mutex> lock(nodes_by_name_mutex_);
  return nodes_by_name_.count(node_name) != 0;
}

void
RosAbstraction::clear_rclcpp_nodes()
{
  std::lock_guard<std::mutex> lock(nodes_by_name_mutex_);
  nodes_by_name_.clear();
}

}  // namespace ros_integration
}  // namespace rviz_common
