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

#ifndef RVIZ_COMMON__ROS_INTEGRATION__ROS_NODE_STORAGE_HPP_
#define RVIZ_COMMON__ROS_INTEGRATION__ROS_NODE_STORAGE_HPP_

#include <map>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "./ros_node_storage_iface.hpp"

namespace rviz_common
{
namespace ros_integration
{

class RosNodeStorage : public RosNodeStorageIface
{
public:
  /// Store an rclcpp node shared pointer in the internal storage by a given name.
  /**
   * If the key is already in use, the new node shared pointer overwrites the
   * existing node shared pointer stored as the value for that key.
   *
   * \param node_name name to be used as the key for the rclcpp node
   * \param node the rclcpp node to be stored
   */
  void
  store_rclcpp_node_by_name(
    const std::string & node_name,
    std::shared_ptr<rclcpp::Node> node) override;

  /// Return the rclcpp node shared pointer for the given node name if found, else nullptr.
  /**
   * \param node_name the name of the rclcpp node to get
   * \returns the rclcpp node shared pointer for the given name, else nullptr
   */
  std::shared_ptr<rclcpp::Node>
  get_rclcpp_node_by_name(const std::string & node_name) override;

  /// Check if there exists an rclcpp node for the given name.
  /**
   * \param node_name the name of the node to check for
   * \return true if exists, otherwise false
   */
  bool
  has_rclcpp_node_by_name(const std::string & node_name) override;

  /// Clear the stored nodes, allowing them to go out of scope.
  /**
   * This function is primarily used by shutdown to clean up the nodes created.
   */
  void
  clear_rclcpp_nodes() override;

private:
  static std::map<std::string, rclcpp::Node::SharedPtr> nodes_by_name_;
  static std::mutex nodes_by_name_mutex_;
};

}  // namespace ros_integration
}  // namespace rviz_common

#endif  // RVIZ_COMMON__ROS_INTEGRATION__ROS_NODE_STORAGE_HPP_
