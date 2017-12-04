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

#ifndef RVIZ_COMMON__ROS_INTEGRATION__ROS_ABSTRACTION_HPP_
#define RVIZ_COMMON__ROS_INTEGRATION__ROS_ABSTRACTION_HPP_

#include <map>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "ros_abstraction_iface.hpp"

namespace rviz_common
{
namespace ros_integration
{

class RosAbstraction : public RosAbstractionIface
{
public:
  // TODO(wjwwood): Figure out which exceptions can be raised and document them
  //                consider consolidating all possible exceptions to a few
  //                exceptions defined in this library, to avoid inconsistent
  //                exceptions based on the underly ROS version.
  //                Also define an exception for repeated calls to this function
  //                under ROS 1, which will not be allowed.
  /// Initialize ROS, create the ROS node, return the ROS node name.
  /**
   * argc and argv maybe mutate to remove any command line arguments consumed by ROS.
   *
   * The returned ROS node name will be used in other API calls to reference the
   * correct node.
   * In ROS 2, this function maybe called multiple times and so the ROS node name
   * will be the unique "key" used to operate on the correct node indirectly.
   * In ROS 1 this will raise an exception.
   *
   * \param argc number of elements in argv
   * \param argv command line arguments as an array of c-string
   * \param name desired node name, or base node name if using an anonymous name
   * \param anonymous_name if true then the ROS node name will be randomized
   * \return name of the resulting ROS node
   */
  std::string
  init(int argc, char ** argv, const std::string & name, bool anonymous_name = true) override;

  /// Check if ROS is "ok" or not, usually if ROS has been shutdown or not.
  /**
   * \param node_name the name of the node returned by ros_integration::init()
   * \return true if ok, otherwise false
   */
  bool
  ok(const std::string & node_name) override;

  /// Shutdown ROS.
  /**
   * This will also destroy any nodes which were created with
   * ros_integration::init().
   */
  void
  shutdown() override;

private:
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
    const std::shared_ptr<rclcpp::Node> & node);

  /// Return the rclcpp node shared pointer for the given node name if found, else nullptr.
  /**
   * \param node_name the name of the rclcpp node to get
   * \returns the rclcpp node shared pointer for the given name, else nullptr
   */
  std::shared_ptr<rclcpp::Node>
  get_rclcpp_node_by_name(const std::string & node_name);

  /// Check if there exists an rclcpp node for the given name.
  /**
   * \param node_name the name of the node to check for
   * \return true if exists, otherwise false
   */
  bool
  has_rclcpp_node_by_name(const std::string & node_name);

  /// Clear the stored nodes, allowing them to go out of scope.
  /**
   * This function is primarily used by shutdown to clean up the nodes created.
   */
  void
  clear_rclcpp_nodes();

  std::map<std::string, rclcpp::Node::SharedPtr> nodes_by_name_;
  std::mutex nodes_by_name_mutex_;
};

}  // namespace ros_integration
}  // namespace rviz_common

#endif  // RVIZ_COMMON__ROS_INTEGRATION__ROS_ABSTRACTION_HPP_
