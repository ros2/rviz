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

#ifndef RVIZ_COMMON__ROS_INTEGRATION__ROS_CLIENT_ABSTRACTION_HPP_
#define RVIZ_COMMON__ROS_INTEGRATION__ROS_CLIENT_ABSTRACTION_HPP_

#include <memory>
#include <string>

#include "ros_client_abstraction_iface.hpp"
#include "ros_node_abstraction_iface.hpp"
#include "rviz_common/visibility_control.hpp"

namespace rviz_common
{
namespace ros_integration
{

class RosClientAbstraction : public RosClientAbstractionIface
{
public:
  RVIZ_COMMON_PUBLIC
  RosClientAbstraction();

  // TODO(wjwwood): Figure out which exceptions can be raised and document them
  //                consider consolidating all possible exceptions to a few
  //                exceptions defined in this library, to avoid inconsistent
  //                exceptions based on the underly ROS version.
  //                Also define an exception for repeated calls to this function
  //                under ROS 1, which will not be allowed.
  /// Initialize ROS, create the ROS node and return it (inside a weak pointer).
  /**
   * argc and argv maybe mutate to remove any command line arguments consumed by ROS.
   *
   * The returned ROS node will be used for other API calls.
   * This function throws if called multiple times.
   *
   * \param argc number of elements in argv
   * \param argv command line arguments as an array of c-string
   * \param name desired node name, or base node name if using an anonymous name
   * \param anonymous_name if true then the ROS node name will be randomized
   * \return weak pointer of the created ros node
   */
  RVIZ_COMMON_PUBLIC
  RosNodeAbstractionIface::WeakPtr
  init(int argc, char ** argv, const std::string & name, bool anonymous_name) override;

  /// Check if ROS is "ok" or not, usually if ROS has been shutdown or not.
  /**
   * \param node_name the name of the node returned by ros_integration::init()
   * \return true if ok, otherwise false
   */
  RVIZ_COMMON_PUBLIC
  bool
  ok() override;

  /// Shutdown ROS.
  /**
   * This will also destroy any nodes which were created with
   * ros_integration::init().
   */
  RVIZ_COMMON_PUBLIC
  void
  shutdown() override;

private:
  std::shared_ptr<RosNodeAbstractionIface> rviz_ros_node_;
};

}  // namespace ros_integration
}  // namespace rviz_common

#endif  // RVIZ_COMMON__ROS_INTEGRATION__ROS_CLIENT_ABSTRACTION_HPP_
