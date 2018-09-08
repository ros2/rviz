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

bool TFWrapper::canTransform(
  const std::string & fixed_frame,
  const std::string & frame,
  tf2::TimePoint time,
  std::string & error)
{
  return buffer_->canTransform(fixed_frame, frame, time, &error);
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
  initializeBuffer(clock, using_dedicated_thread);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
    *buffer_, rviz_ros_node.lock()->get_raw_node(), false);
}

void TFWrapper::initializeBuffer(rclcpp::Clock::SharedPtr clock, bool using_dedicated_thread)
{
  buffer_ = std::make_shared<tf2_ros::Buffer>(clock);
  buffer_->setUsingDedicatedThread(using_dedicated_thread);
}

void TFWrapper::clear()
{
  buffer_->clear();
}

}  // namespace transformation
}  // namespace rviz_default_plugins
