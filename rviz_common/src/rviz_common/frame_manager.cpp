/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * Copyright (c) 2017, Open Source Robotics Foundation, Inc.
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

#include "frame_manager.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/clock.hpp"
#include "std_msgs/msg/float32.hpp"

#include "rviz_common/display.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/msg_conversions.hpp"
#include "rviz_common/properties/property.hpp"
#include "rviz_common/transformation/tf2_helpers/tf2_conversion_helpers.hpp"

namespace rviz_common
{

FrameManager::FrameManager(
  rclcpp::Clock::SharedPtr clock, std::shared_ptr<transformation::FrameTransformer> transformer)
: transformer_(transformer), sync_time_(0), clock_(clock)
{
  setSyncMode(SyncOff);
  setPause(false);
}

void FrameManager::update()
{
  std::lock_guard<std::mutex> lock(cache_mutex_);
  if (!pause_) {
    cache_.clear();
  }

  if (!pause_) {
    switch (sync_mode_) {
      case SyncOff:
        sync_time_ = clock_->now();
        break;
      case SyncExact:
        break;
      case SyncApprox:
        // adjust current time offset to sync source
        current_delta_ = static_cast<uint64_t>(0.7 * current_delta_ + 0.3 * sync_delta_);
        sync_time_ = rclcpp::Time(
          clock_->now().nanoseconds() - current_delta_, clock_->get_clock_type());
        break;
    }
  }
}

void FrameManager::setFixedFrame(const std::string & frame)
{
  bool should_emit = false;
  {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    if (fixed_frame_ != frame) {
      fixed_frame_ = frame;
      cache_.clear();
      should_emit = true;
    }
  }
  if (should_emit) {
    // This emission must be kept outside of the mutex lock to avoid deadlocks.
    emit fixedFrameChanged();
  }
}

void FrameManager::setPause(bool pause)
{
  pause_ = pause;
}

bool FrameManager::getPause()
{
  return pause_;
}

FrameManager::SyncMode FrameManager::getSyncMode()
{
  return sync_mode_;
}

void FrameManager::setSyncMode(SyncMode mode)
{
  sync_mode_ = mode;
  sync_time_ = rclcpp::Time(0, 0, clock_->get_clock_type());
  current_delta_ = 0;
  sync_delta_ = 0;
}

void FrameManager::syncTime(rclcpp::Time time)
{
  switch (sync_mode_) {
    case SyncOff:
      break;
    case SyncExact:
      sync_time_ = time;
      break;
    case SyncApprox:
      if (time == rclcpp::Time(0, 0, clock_->get_clock_type())) {
        sync_delta_ = 0;
        return;
      }
      // avoid exception due to negative time
      if (clock_->now() >= time) {
        sync_delta_ = (clock_->now() - time).nanoseconds();
      } else {
        setSyncMode(SyncApprox);
      }
      break;
  }
}

rclcpp::Time FrameManager::getTime()
{
  return sync_time_;
}

bool FrameManager::adjustTime(const std::string & frame, rclcpp::Time & time)
{
  // we only need to act if we get a zero timestamp, which means "latest"
  if (time != rclcpp::Time(0, 0, clock_->get_clock_type())) {
    return true;
  }

  switch (sync_mode_) {
    case SyncOff:
      break;
    case SyncExact:
      time = sync_time_;
      break;
    case SyncApprox:
      {
        std::string error_message;
        // try to get the time from the latest available transformation
        if (transformer_->canTransform(
            fixed_frame_, frame, tf2::TimePointZero, &error_message))
        {
          time = sync_time_;
        }
      }
      break;
  }
  return true;
}

bool FrameManager::getTransform(
  const std::string & frame,
  rclcpp::Time time,
  Ogre::Vector3 & position,
  Ogre::Quaternion & orientation)
{
  if (!adjustTime(frame, time)) {
    return false;
  }

  std::lock_guard<std::mutex> lock(cache_mutex_);

  position = Ogre::Vector3(9999999, 9999999, 9999999);
  orientation = Ogre::Quaternion::IDENTITY;

  if (fixed_frame_.empty()) {
    return false;
  }

  auto it = cache_.find(CacheKey(frame, time));
  if (it != cache_.end()) {
    position = it->second.position;
    orientation = it->second.orientation;
    return true;
  }

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0;
  pose.position.y = 0;
  pose.position.z = 0;
  pose.orientation.w = 1.0f;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;

  if (!transform(frame, time, pose, position, orientation)) {
    return false;
  }

  cache_.insert(std::make_pair(CacheKey(frame, time), CacheEntry(position, orientation)));

  return true;
}

bool FrameManager::transform(
  const std::string & frame,
  rclcpp::Time time,
  const geometry_msgs::msg::Pose & pose_msg,
  Ogre::Vector3 & position,
  Ogre::Quaternion & orientation)
{
  if (!adjustTime(frame, time)) {
    return false;
  }

  position = Ogre::Vector3::ZERO;
  orientation = Ogre::Quaternion::IDENTITY;

  geometry_msgs::msg::PoseStamped pose_in;
  pose_in.header.stamp = time;
  pose_in.header.frame_id = frame;
  // TODO(wjwwood): figure out where the `/` is coming from and remove it
  //                also consider warning the user in the GUI about this...
  if (pose_in.header.frame_id[0] == '/') {
    pose_in.header.frame_id = pose_in.header.frame_id.substr(1);
  }
  pose_in.pose = pose_msg;

  // TODO(wjwwood): figure out where the `/` is coming from and remove it
  //                also consider warning the user in the GUI about this...
  std::string stripped_fixed_frame = fixed_frame_;
  if (stripped_fixed_frame[0] == '/') {
    stripped_fixed_frame = stripped_fixed_frame.substr(1);
  }

  geometry_msgs::msg::PoseStamped pose_out;
  try {
    pose_out = transformer_->transform(pose_in, stripped_fixed_frame);
  } catch (const transformation::FrameTransformerException & exception) {
    (void) exception;
    return false;
  }

  position = rviz_common::pointMsgToOgre(pose_out.pose.position);
  orientation = rviz_common::quaternionMsgToOgre(pose_out.pose.orientation);
  return true;
}

bool FrameManager::frameHasProblems(const std::string & frame, std::string & error)
{
  return transformer_->frameHasProblems(frame, error);
}

bool FrameManager::transformHasProblems(
  const std::string & frame,
  rclcpp::Time time,
  std::string & error)
{
  if (!adjustTime(frame, time)) {
    return false;
  }

  return !transformer_->canTransform(
    fixed_frame_, frame, transformation::tf2_helpers::toTf2TimePoint(time), &error);
}

const std::string & FrameManager::getFixedFrame()
{
  return fixed_frame_;
}

transformation::TransformationLibraryConnector::WeakPtr FrameManager::getConnector()
{
  return transformer_->getConnector();
}

std::shared_ptr<transformation::FrameTransformer> FrameManager::getTransformer()
{
  return transformer_;
}

std::vector<std::string> FrameManager::getAllFrameNames()
{
  return transformer_->getAllFrameNames();
}

void FrameManager::clear()
{
  transformer_->clear();
}

bool FrameManager::anyTransformationDataAvailable()
{
  auto frames = transformer_->getAllFrameNames();
  return !frames.empty();
}

void FrameManager::setTransformerPlugin(
  std::shared_ptr<transformation::FrameTransformer> transformer)
{
  transformer_ = transformer;
}

}  // namespace rviz_common
