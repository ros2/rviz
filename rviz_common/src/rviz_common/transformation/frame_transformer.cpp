/*
 * Copyright (c) 2019, Open Source Robotics Foundation, Inc.
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

#include "rviz_common/transformation/frame_transformer.hpp"

#include <tf2/exceptions.h>

#include <memory>
#include <string>

#include "rviz_common/transformation/ros_helpers/ros_conversion_helpers.hpp"
#include "rviz_common/transformation/tf2_helpers/tf2_conversion_helpers.hpp"

namespace rviz_common
{
namespace transformation
{

geometry_msgs::msg::TransformStamped FrameTransformer::lookupTransform(
  const std::string & target_frame,
  const std::string & source_frame,
  const tf2::TimePoint & time) const
{
  return ros_helpers::toRosTransformStamped(
    lookupTransform(target_frame, source_frame, tf2_helpers::fromTf2TimePoint(time)));
}

geometry_msgs::msg::TransformStamped FrameTransformer::lookupTransform(
  const std::string & target_frame,
  const tf2::TimePoint & target_time,
  const std::string & source_frame,
  const tf2::TimePoint & source_time,
  const std::string & fixed_frame) const
{
  return ros_helpers::toRosTransformStamped(
    lookupTransform(
      target_frame,
      tf2_helpers::fromTf2TimePoint(target_time),
      source_frame,
      tf2_helpers::fromTf2TimePoint(source_time),
      fixed_frame));
}

bool FrameTransformer::canTransform(
  const std::string & target_frame,
  const std::string & source_frame,
  const tf2::TimePoint & time,
  std::string * error_msg) const
{
  std::string error_string;
  const bool result = canTransform(
    target_frame,
    source_frame,
    tf2_helpers::fromTf2TimePoint(time),
    error_string);
  if (error_msg) {
    *error_msg = error_string;
  }
  return result;
}

bool FrameTransformer::canTransform(
  const std::string & target_frame,
  const tf2::TimePoint & target_time,
  const std::string & source_frame,
  const tf2::TimePoint & source_time,
  const std::string & fixed_frame,
  std::string * error_msg) const
{
  std::string error_string;
  const bool result = canTransform(
    target_frame,
    tf2_helpers::fromTf2TimePoint(target_time),
    source_frame,
    tf2_helpers::fromTf2TimePoint(source_time),
    fixed_frame,
    error_string);
  if (error_msg) {
    *error_msg = error_string;
  }
  return result;
}

tf2_ros::TransformStampedFuture FrameTransformer::waitForTransform(
  const std::string & target_frame,
  const std::string & source_frame,
  const tf2::TimePoint & time,
  const tf2::Duration & timeout,
  tf2_ros::TransformReadyCallback callback)
{
  auto tf2_promise = std::make_shared<std::promise<geometry_msgs::msg::TransformStamped>>();
  tf2_ros::TransformStampedFuture tf2_future(tf2_promise->get_future());
  waitForTransform(
    target_frame,
    source_frame,
    tf2_helpers::fromTf2TimePoint(time),
    timeout,
    [tf2_promise, tf2_future, callback](const TransformStampedFuture & future)
    {
      try {
        TransformStamped transform_stamped = future.get();
        const auto ros_transform_stamped = ros_helpers::toRosTransformStamped(transform_stamped);
        tf2_promise->set_value(ros_transform_stamped);
      } catch (const tf2::LookupException & ex) {
        tf2_promise->set_exception(std::make_exception_ptr<tf2::LookupException>(ex));
      }
      callback(tf2_future);
    });
  return tf2_future;
}

}  // namespace transformation
}  // namespace rviz_common
