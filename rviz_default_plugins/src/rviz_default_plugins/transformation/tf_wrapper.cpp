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

#include "rviz_common/tf_wrapper.hpp"

#include <memory>
#include <string>
#include <vector>

namespace rviz_common
{

TFWrapper::TFWrapper(std::shared_ptr<tf2_ros::Buffer> buffer, bool using_dedicated_thread)
: buffer_(buffer), tf_listener_(nullptr)
{
  buffer_->setUsingDedicatedThread(using_dedicated_thread);
}

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

void TFWrapper::setListener(std::shared_ptr<tf2_ros::TransformListener> tf_listener)
{
  tf_listener_ = tf_listener;
}

void TFWrapper::clear()
{
  buffer_->clear();
}

}  // namespace rviz_common
