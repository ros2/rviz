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

#ifndef RVIZ_COMMON__TRANSFORMATION__IDENTITY_FRAME_TRANSFORMER_HPP_
#define RVIZ_COMMON__TRANSFORMATION__IDENTITY_FRAME_TRANSFORMER_HPP_

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "rviz_common/transformation/frame_transformer.hpp"
#include "rviz_common/transformation/structs.hpp"
#include "rviz_common/visibility_control.hpp"

namespace rviz_common
{
namespace transformation
{

/// A trivial subclass of FrameTransformer, that always uses the identity transform.
class RVIZ_COMMON_PUBLIC IdentityFrameTransformer : public FrameTransformer
{
public:
  IdentityFrameTransformer() = default;
  ~IdentityFrameTransformer() override = default;

  void
  initialize(
    ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node,
    rclcpp::Clock::SharedPtr clock) override;

  void
  clear() override;

  std::vector<std::string>
  getAllFrameNames() override;

  transformation::PoseStamped
  transform(
    // NOLINT (this is not std::transform)
    const transformation::PoseStamped & pose_in,
    const std::string & target_frame) override;

  bool
  transformIsAvailable(
    const std::string & target_frame,
    const std::string & source_frame) override;

  bool
  transformHasProblems(
    const std::string & source_frame,
    const std::string & target_frame,
    const rclcpp::Time & time,
    std::string & error) override;

  bool
  frameHasProblems(const std::string & frame, std::string & error) override;

  TransformationLibraryConnector::WeakPtr
  getConnector() override;

#if 0
  void
  waitForValidTransform(
    std::string target_frame,
    std::string source_frame,
    rclcpp::Time time,
    rclcpp::Duration timeout,
    std::function<void(void)> callback) override;
#endif

private:
  bool
  quaternionIsValid(transformation::Quaternion quaternion);
};

}  // namespace transformation
}  // namespace rviz_common

#endif  // RVIZ_COMMON__TRANSFORMATION__IDENTITY_FRAME_TRANSFORMER_HPP_
