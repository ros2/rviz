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

#include "identity_frame_transformer.hpp"

#include <future>
#include <memory>
#include <string>
#include <vector>

#include "rviz_common/transformation/tf2_helpers/tf2_conversion_helpers.hpp"

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

geometry_msgs::msg::PoseStamped IdentityFrameTransformer::transform(
  const geometry_msgs::msg::PoseStamped & pose_in, const std::string & target_frame)
{
  (void) target_frame;
  geometry_msgs::msg::PoseStamped pose_out = pose_in;

  if (quaternionIsValid(pose_out.pose.orientation)) {
    return pose_out;
  }
  pose_out.pose.orientation.w = 1;
  return pose_out;
}

TransformationLibraryConnector::WeakPtr IdentityFrameTransformer::getConnector()
{
  return std::weak_ptr<TransformationLibraryConnector>();
}

bool IdentityFrameTransformer::frameHasProblems(
  const std::string & frame, std::string & error) const
{
  (void) frame;
  (void) error;

  return false;
}

void IdentityFrameTransformer::clear() {}

geometry_msgs::msg::TransformStamped IdentityFrameTransformer::lookupTransform(
  const std::string & target_frame,
  const std::string & source_frame,
  const tf2::TimePoint & time) const
{
  geometry_msgs::msg::TransformStamped transform;
  transform.child_frame_id = target_frame;
  transform.header = transformation::tf2_helpers::createHeader(time, source_frame);
  transform.transform.translation.x = 0.0;
  transform.transform.translation.y = 0.0;
  transform.transform.translation.z = 0.0;
  transform.transform.rotation.x = 0.0;
  transform.transform.rotation.y = 0.0;
  transform.transform.rotation.z = 0.0;
  transform.transform.rotation.w = 1.0;
  return transform;
}

geometry_msgs::msg::TransformStamped IdentityFrameTransformer::lookupTransform(
  const std::string & target_frame,
  const tf2::TimePoint & target_time,
  const std::string & source_frame,
  const tf2::TimePoint & source_time,
  const std::string & fixed_frame) const
{
  (void) fixed_frame;
  (void) source_time;

  geometry_msgs::msg::TransformStamped transform;
  transform.child_frame_id = target_frame;
  transform.header = transformation::tf2_helpers::createHeader(target_time, source_frame);
  transform.transform.translation.x = 0.0;
  transform.transform.translation.y = 0.0;
  transform.transform.translation.z = 0.0;
  transform.transform.rotation.x = 0.0;
  transform.transform.rotation.y = 0.0;
  transform.transform.rotation.z = 0.0;
  transform.transform.rotation.w = 1.0;
  return transform;
}

bool IdentityFrameTransformer::canTransform(
  const std::string & target_frame,
  const std::string & source_frame,
  const tf2::TimePoint & time,
  std::string * error_msg) const
{
  (void) target_frame;
  (void) source_frame;
  (void) time;
  (void) error_msg;

  return true;
}

bool IdentityFrameTransformer::canTransform(
  const std::string & target_frame,
  const tf2::TimePoint & target_time,
  const std::string & source_frame,
  const tf2::TimePoint & source_time,
  const std::string & fixed_frame,
  std::string * error_msg) const
{
  (void) target_frame;
  (void) target_time;
  (void) source_frame;
  (void) source_time;
  (void) fixed_frame;
  (void) error_msg;

  return true;
}

std::vector<std::string> IdentityFrameTransformer::getAllFrameNames() const
{
  return {""};
}

tf2_ros::TransformStampedFuture IdentityFrameTransformer::waitForTransform(
  const std::string & target_frame,
  const std::string & source_frame,
  const tf2::TimePoint & time,
  const tf2::Duration & timeout,
  tf2_ros::TransformReadyCallback callback)
{
  (void) timeout;

  geometry_msgs::msg::TransformStamped transform;
  transform.child_frame_id = target_frame;
  transform.header = transformation::tf2_helpers::createHeader(time, source_frame);
  transform.transform.translation.x = 0.0;
  transform.transform.translation.y = 0.0;
  transform.transform.translation.z = 0.0;
  transform.transform.rotation.x = 0.0;
  transform.transform.rotation.y = 0.0;
  transform.transform.rotation.z = 0.0;
  transform.transform.rotation.w = 1.0;

  std::promise<geometry_msgs::msg::TransformStamped> promise;
  tf2_ros::TransformStampedFuture future(promise.get_future());
  promise.set_value(transform);
  callback(future);
  return future;
}

void
IdentityFrameTransformer::cancel(const tf2_ros::TransformStampedFuture & ts_future)
{
  (void) ts_future;
}

bool IdentityFrameTransformer::quaternionIsValid(geometry_msgs::msg::Quaternion quaternion)
{
  return quaternion.w + quaternion.x + quaternion.y + quaternion.z != 0;
}

}  // namespace transformation
}  // namespace rviz_common
