/*
 * Copyright (c) 2018, Bosch Software Innovations GmbH.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "./identity_frame_transformer.hpp"

#include <memory>
#include <string>
#include <vector>

namespace rviz_common
{
namespace transformation
{

void IdentityFrameTransformer::initialize(
  ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node, rclcpp::Clock::SharedPtr clock)
{
  (void) rviz_ros_node;
  (void) clock;
}

void IdentityFrameTransformer::clear() {}

std::vector<std::string> IdentityFrameTransformer::getAllFrameNames()
{
  return {""};
}

transformation::PoseStamped IdentityFrameTransformer::transform(
  const transformation::PoseStamped & pose_in, const std::string & target_frame)
{
  (void) target_frame;
  transformation::PoseStamped pose_out = pose_in;

  if (quaternionIsValid(pose_out.orientation)) {
    return pose_out;
  }
  pose_out.orientation.w = 1;
  return pose_out;
}

bool IdentityFrameTransformer::transformIsAvailable(
  const std::string & target_frame, const std::string & source_frame)
{
  (void) target_frame;
  (void) source_frame;

  return true;
}

bool IdentityFrameTransformer::transformHasProblems(
  const std::string & source_frame,
  const std::string & target_frame,
  const rclcpp::Time & time,
  std::string & error)
{
  (void) source_frame;
  (void) target_frame;
  (void) time;
  (void) error;

  return false;
}

bool IdentityFrameTransformer::frameHasProblems(const std::string & frame, std::string & error)
{
  (void) frame;
  (void) error;

  return false;
}

TransformationLibraryConnector::WeakPtr IdentityFrameTransformer::getConnector()
{
  return std::weak_ptr<TransformationLibraryConnector>();
}

bool IdentityFrameTransformer::quaternionIsValid(transformation::Quaternion quaternion)
{
  return quaternion.w + quaternion.x + quaternion.y + quaternion.z != 0;
}

}  // namespace transformation
}  // namespace rviz_common
