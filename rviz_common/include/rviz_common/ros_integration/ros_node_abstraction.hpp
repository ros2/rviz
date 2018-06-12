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

#ifndef RVIZ_COMMON__ROS_INTEGRATION__ROS_NODE_ABSTRACTION_HPP_
#define RVIZ_COMMON__ROS_INTEGRATION__ROS_NODE_ABSTRACTION_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"
#include "rviz_common/visibility_control.hpp"

namespace rviz_common
{
namespace ros_integration
{

class RosNodeAbstraction : public RosNodeAbstractionIface
{
public:
  RVIZ_COMMON_PUBLIC
  RosNodeAbstraction() = delete;

  /// Creates a ros node with the given name
  /**
   * Internally a rclcpp::Node is created.
   * If a rclcpp::Node with the given name already exists a node abstraction for this already
   * existing node is created.
   *
   * \param node_name name of the node to create
   */
  RVIZ_COMMON_PUBLIC
  explicit RosNodeAbstraction(const std::string & node_name);

  /// Returns the name of the ros node
  /**
   * The returned node name is what was given as constructor argument.
   *
   * \return the name of the node
   */
  RVIZ_COMMON_PUBLIC
  std::string get_node_name() const override;

  /// Return a map with topic names mapped to a list of types for that topic.
  /**
   * The node name is what was given when initializing this API.
   *
   * \return map of topic names and their types
   */
  RVIZ_COMMON_PUBLIC
  std::map<std::string, std::vector<std::string>>
  get_topic_names_and_types() const override;

  // TODO(wjwwood): think about a suitable way to extend the abstraction to also cover subscriptions
  RVIZ_COMMON_PUBLIC
  rclcpp::Node::SharedPtr
  get_raw_node() override
  {
    return raw_node_;
  }

private:
  rclcpp::Node::SharedPtr raw_node_;
};

}  // namespace ros_integration
}  // namespace rviz_common

#endif  // RVIZ_COMMON__ROS_INTEGRATION__ROS_NODE_ABSTRACTION_HPP_
