/*
 * Copyright (c) 2018, Bosch Software Innovations GmbH.
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
 *     * Neither the name of the copyright holder nor the names of its contributors
 *       may be used to endorse or promote products derived from
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

#include "rviz_visual_testing_framework/internal/transform_message_creator.hpp"

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "tf2/LinearMath/Quaternion.h"

geometry_msgs::msg::TransformStamped createStaticTransformMessageFor(
  std::string header_frame_id, std::string child_frame_id)
{
  return createStaticTransformMessageFor(
    header_frame_id, child_frame_id,
    0, 0, 0,
    0, 0, 0);
}

geometry_msgs::msg::TransformStamped createStaticTransformMessageFor(
  std::string header_frame_id, std::string child_frame_id,
  int x, int y, int z,
  int roll, int pitch, int yaw)
{
  geometry_msgs::msg::TransformStamped message;

  message.transform.translation.x = x;
  message.transform.translation.y = y;
  message.transform.translation.z = z;

  tf2::Quaternion quat;
  quat.setRPY(roll, pitch, yaw);
  message.transform.rotation.x = quat.x();
  message.transform.rotation.y = quat.y();
  message.transform.rotation.z = quat.z();
  message.transform.rotation.w = quat.w();

  message.header.stamp = rclcpp::Clock().now();
  message.header.frame_id = header_frame_id;
  message.child_frame_id = child_frame_id;

  return message;
}
