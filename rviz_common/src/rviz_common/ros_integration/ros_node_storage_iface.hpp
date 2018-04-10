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

#ifndef RVIZ_COMMON__ROS_INTEGRATION__ROS_NODE_STORAGE_IFACE_HPP_
#define RVIZ_COMMON__ROS_INTEGRATION__ROS_NODE_STORAGE_IFACE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace rviz_common
{
namespace ros_integration
{

class RosNodeStorageIface
{
public:
  virtual ~RosNodeStorageIface() = default;

  virtual void
  store_rclcpp_node_by_name(
    const std::string & node_name,
    std::shared_ptr<rclcpp::Node> node) = 0;

  virtual std::shared_ptr<rclcpp::Node>
  get_rclcpp_node_by_name(const std::string & node_name) = 0;

  virtual bool
  has_rclcpp_node_by_name(const std::string & node_name) = 0;

  virtual void
  clear_rclcpp_nodes() = 0;
};

}  // namespace ros_integration
}  // namespace rviz_common

#endif  // RVIZ_COMMON__ROS_INTEGRATION__ROS_NODE_STORAGE_IFACE_HPP_
