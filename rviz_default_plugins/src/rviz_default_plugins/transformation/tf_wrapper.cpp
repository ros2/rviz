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

#include "rviz_default_plugins/transformation/tf_wrapper.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace rviz_default_plugins
{
namespace transformation
{

TFWrapper::TFWrapper()
: buffer_(nullptr), tf_listener_(nullptr)
{}

void TFWrapper::transform(
  const geometry_msgs::msg::PoseStamped & pose_in,
  geometry_msgs::msg::PoseStamped & pose_out,
  const std::string & frame)
{
  buffer_->transform(pose_in, pose_out, frame);
}

geometry_msgs::msg::TransformStamped TFWrapper::lookupTransform(
  const std::string & target_frame,
  const std::string & source_frame,
  const tf2::TimePoint & time)
{
  return buffer_->lookupTransform(target_frame, source_frame, time);
}

geometry_msgs::msg::TransformStamped TFWrapper::lookupTransform(
  const std::string & target_frame,
  const tf2::TimePoint & target_time,
  const std::string & source_frame,
  const tf2::TimePoint & source_time,
  const std::string & fixed_frame)
{
  return buffer_->lookupTransform(
    target_frame, target_time, source_frame, source_time, fixed_frame);
}

bool TFWrapper::canTransform(
  const std::string & target_frame,
  const std::string & source_frame,
  tf2::TimePoint time,
  std::string & error)
{
  return buffer_->canTransform(target_frame, source_frame, time, &error);
}

bool TFWrapper::canTransform(
  const std::string & target_frame,
  const tf2::TimePoint & target_time,
  const std::string & source_frame,
  const tf2::TimePoint & source_time,
  const std::string & fixed_frame,
  std::string & error)
{
  return buffer_->canTransform(
    target_frame, target_time, source_frame, source_time, fixed_frame, &error);
}

tf2_ros::TransformStampedFuture TFWrapper::waitForTransform(
  const std::string & target_frame,
  const std::string & source_frame,
  const tf2::TimePoint & time,
  const tf2::Duration & timeout,
  tf2_ros::TransformReadyCallback callback)
{
  return buffer_->waitForTransform(target_frame, source_frame, time, timeout, callback);
}

void TFWrapper::cancel(
  const tf2_ros::TransformStampedFuture & ts_future)
{
  return buffer_->cancel(ts_future);
}

std::vector<std::string> TFWrapper::getFrameStrings()
{
  std::vector<std::string> frames;
  buffer_->_getFrameStrings(frames);
  return frames;
}

bool TFWrapper::frameExists(const std::string & frame)
{
  return buffer_->_frameExists(frame);
}

std::shared_ptr<tf2_ros::Buffer> TFWrapper::getBuffer()
{
  return buffer_;
}

void TFWrapper::initialize(
  rclcpp::Clock::SharedPtr clock,
  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node,
  bool using_dedicated_thread)
{
  initializeBuffer(clock, rviz_ros_node.lock()->get_raw_node(), using_dedicated_thread);
  if (using_dedicated_thread) {
    // TODO(pull/551): The TransformListener needs very quick spinning so it uses its own node
    // here. Remove this in favor of a multithreaded spinner and ensure that the listener callback
    // queue does not fill up.
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
      *buffer_, true);
  } else {
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
      *buffer_, rviz_ros_node.lock()->get_raw_node(), false);
  }
}

void TFWrapper::initializeBuffer(
  rclcpp::Clock::SharedPtr clock, rclcpp::Node::SharedPtr node, bool using_dedicated_thread)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = "Configure the rviz tf buffer cache time [ms].";
  descriptor.read_only = true;
  int64_t cache_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    tf2::BUFFER_CORE_DEFAULT_CACHE_TIME).count();
  try {
    cache_time_ms = node->declare_parameter<int64_t>(
      "tf_buffer_cache_time_ms",
      cache_time_ms,
      descriptor);
  } catch (rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
    node->get_parameter<int64_t>("tf_buffer_cache_time_ms", cache_time_ms);
  }
  if (cache_time_ms < 0) {
    RCLCPP_WARN(
      node->get_logger(),
      "Specified parameter 'tf_buffer_cache_time_ms' is < 0, using the default tf buffer"
      " cache time");
  }
  std::chrono::milliseconds cache_time{cache_time_ms};
  buffer_ = std::make_shared<tf2_ros::Buffer>(clock, cache_time);
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    node->get_node_base_interface(), node->get_node_timers_interface());
  buffer_->setCreateTimerInterface(timer_interface);
  buffer_->setUsingDedicatedThread(using_dedicated_thread);
}

void TFWrapper::clear()
{
  buffer_->clear();
}

}  // namespace transformation
}  // namespace rviz_default_plugins
