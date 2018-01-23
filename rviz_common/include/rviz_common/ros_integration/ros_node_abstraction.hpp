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

#include "rviz_common/visibility_control.hpp"

#include "./ros_node_abstraction_iface.hpp"

namespace rviz_common
{
namespace ros_integration
{

// forward declaration so ros node storage headers can remain private
class RosNodeStorageIface;

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

  /// Creates a ros node with the given name using a specific storage. For testing use only.
  /**
   * Internally a rclcpp::Node is created.
   * If a rclcpp::Node with the given name already exists a node abstraction for this already
   * existing node is created.
   *
   * \param node_name name of the node to create
   * \param ros_node_storage storage handling for the internal rclcpp::Node
   */
  RVIZ_COMMON_PUBLIC
  RosNodeAbstraction(
    const std::string & node_name,
    std::shared_ptr<RosNodeStorageIface> ros_node_storage);

  /// Returns the name of the ros node
  /**
   * The returned node name is what was given as constructor argument.
   *
   * \return the name of the node
   */
  RVIZ_COMMON_PUBLIC
  std::string get_node_name() override;

  /// Return a map with topic names mapped to a list of types for that topic.
  /**
   * The node name is what was given when initializing this API.
   *
   * \return map of topic names and their types
   */
  RVIZ_COMMON_PUBLIC
  std::map<std::string, std::vector<std::string>>
  get_topic_names_and_types() override;

private:
  std::string node_name_;
  std::shared_ptr<RosNodeStorageIface> ros_node_storage_;
};

}  // namespace ros_integration
}  // namespace rviz_common

#endif  // RVIZ_COMMON__ROS_INTEGRATION__ROS_NODE_ABSTRACTION_HPP_
