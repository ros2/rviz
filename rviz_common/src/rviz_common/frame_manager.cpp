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

#include "./frame_manager.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <utility>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2/buffer_core.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"

#include "./display.hpp"
#include "rviz_common/properties/property.hpp"
#include "rviz_common/logging.hpp"

namespace rviz_common
{

FrameManager::FrameManager(
  std::shared_ptr<tf2_ros::TransformListener> tf,
  std::shared_ptr<tf2_ros::Buffer> buffer)
: sync_time_(0)
{
  if (!tf) {
    // TODO(wjwwood): reenable this when possible (ros2 has no singleton node),
    //                for now just require it to be passed in
    // tf_.reset(new tf2_ros::TransformListener(ros::NodeHandle(), ros::Duration(10 * 60), true));
    throw std::runtime_error("given TransformListener is nullprt");
  } else {
    tf_ = tf;
  }
  buffer_ = buffer;

  setSyncMode(SyncOff);
  setPause(false);
}

FrameManager::~FrameManager()
{
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
        sync_time_ = rclcpp::Time::now();
        break;
      case SyncExact:
        break;
      case SyncApprox:
        // adjust current time offset to sync source
        current_delta_ = 0.7 * current_delta_ + 0.3 * sync_delta_;
        sync_time_ = rclcpp::Time(rclcpp::Time::now().nanoseconds() - current_delta_);
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
  sync_time_ = rclcpp::Time(0);
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
      if (time == rclcpp::Time(0)) {
        sync_delta_ = 0;
        return;
      }
      // avoid exception due to negative time
      if (rclcpp::Time::now() >= time) {
        sync_delta_ = (rclcpp::Time::now() - time).nanoseconds();
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
  if (time != rclcpp::Time()) {
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
        // try to get the time from the latest available transformation
        try {
          auto lastAvailableTransform =
            buffer_->lookupTransform(fixed_frame_, frame, tf2::TimePointZero);
          if (lastAvailableTransform.header.stamp.nanosec > sync_time_.nanoseconds()) {
            time = sync_time_;
          }
        } catch (const tf2::LookupException & exception) {
          RVIZ_COMMON_LOG_ERROR_STREAM("Lookup failed while getting latest time from frame " <<
            frame.c_str() << " to frame " <<
            fixed_frame_.c_str() << ": " <<
            exception.what());
          return false;
        } catch (const tf2::ConnectivityException & exception) {
          RVIZ_COMMON_LOG_ERROR_STREAM("Connection exception getting latest time from frame " <<
            frame.c_str() << " to frame " <<
            fixed_frame_.c_str() << ": " <<
            exception.what());
          return false;
        } catch (const tf2::ExtrapolationException & exception) {
          RVIZ_COMMON_LOG_ERROR_STREAM("Extrapolation exception getting latest time from frame " <<
            frame.c_str() << " to frame " <<
            fixed_frame_.c_str() << ": " <<
            exception.what());
          return false;
        } catch (const tf2::InvalidArgumentException & exception) {
          RVIZ_COMMON_LOG_ERROR_STREAM("Invalid argument exception "
            "getting latest time from frame " <<
            frame.c_str() << " to frame " <<
            fixed_frame_.c_str() << ": " <<
            exception.what());
          return false;
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

  M_Cache::iterator it = cache_.find(CacheKey(frame, time));
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
  geometry_msgs::msg::PoseStamped pose_out;

  // TODO(wjwwood): figure out where the `/` is coming from and remove it
  //                also consider warning the user in the GUI about this...
  std::string stripped_fixed_frame = fixed_frame_;
  if (stripped_fixed_frame[0] == '/') {
    stripped_fixed_frame = stripped_fixed_frame.substr(1);
  }
  // convert pose into new frame
  try {
    buffer_->transform(pose_in, pose_out, stripped_fixed_frame);
  } catch (const tf2::LookupException & exec) {
    // RVIZ_COMMON_LOG_WARNING_STREAM("tf2 lookup exception when transforming: " << exec.what());
    return false;
  } catch (const tf2::ConnectivityException & exec) {
    return false;
  }

  position = Ogre::Vector3(
    pose_out.pose.position.x,
    pose_out.pose.position.y,
    pose_out.pose.position.z);
  orientation = Ogre::Quaternion(
    pose_out.pose.orientation.w,
    pose_out.pose.orientation.x,
    pose_out.pose.orientation.y,
    pose_out.pose.orientation.z);

  return true;
}

bool FrameManager::frameHasProblems(
  const std::string & frame,
  rclcpp::Time time,
  std::string & error)
{
  Q_UNUSED(time);
  if (!buffer_->_frameExists(frame)) {
    error = "Frame [" + frame + "] does not exist";
    if (frame == fixed_frame_) {
      error = "Fixed " + error;
    }
    return true;
  }

  return false;
}

bool FrameManager::transformHasProblems(
  const std::string & frame,
  rclcpp::Time time,
  std::string & error)
{
  if (!adjustTime(frame, time)) {
    return false;
  }

  std::string tf_error;
  tf2::TimePoint tf2_time(std::chrono::nanoseconds(time.nanoseconds()));
  bool transform_succeeded = buffer_->canTransform(fixed_frame_, frame, tf2_time, &tf_error);
  if (transform_succeeded) {
    return false;
  }

  bool ok = true;
  ok = ok && !frameHasProblems(fixed_frame_, time, error);
  ok = ok && !frameHasProblems(frame, time, error);

  if (ok) {
    std::stringstream ss;
    ss << "No transform to fixed frame [" << fixed_frame_ << "].  TF error: [" << tf_error << "]";
    error = ss.str();
    ok = false;
  }

  {
    std::stringstream ss;
    ss << "For frame [" << frame << "]: " << error;
    error = ss.str();
  }

  return !ok;
}

const std::string & FrameManager::getFixedFrame()
{
  return fixed_frame_;
}

tf2_ros::TransformListener * FrameManager::getTFClient()
{
  return tf_.get();
}

const std::shared_ptr<tf2_ros::TransformListener> & FrameManager::getTFClientPtr()
{
  return tf_;
}

std::string getTransformStatusName(const std::string & caller_id)
{
  std::stringstream ss;
  ss << "Transform [sender=" << caller_id << "]";
  return ss.str();
}

#if 0
std::string FrameManager::discoverFailureReason(
  const std::string & frame_id,
  const rclcpp::Time & stamp,
  const std::string & caller_id,
  tf::FilterFailureReason reason)
{
  if (reason == tf::filter_failure_reasons::OutTheBack) {
    std::stringstream ss;
    ss << "Message removed because it is too old (frame=[" << frame_id << "], stamp=[" << stamp <<
      "])";
    return ss.str();
  } else {
    std::string error;
    if (transformHasProblems(frame_id, stamp, error)) {
      return error;
    }
  }

  return "Unknown reason for transform failure";
}
#endif

void FrameManager::messageArrived(
  const std::string & frame_id,
  const rclcpp::Time & stamp,
  const std::string & caller_id,
  Display * display)
{
  Q_UNUSED(frame_id);
  Q_UNUSED(stamp);
  using rviz_common::properties::StatusProperty;
  display->setStatusStd(StatusProperty::Ok, getTransformStatusName(caller_id), "Transform OK");
}

#if 0
void FrameManager::messageFailed(
  const std::string & frame_id,
  const rclcpp::Time & stamp,
  const std::string & caller_id,
  tf::FilterFailureReason reason,
  Display * display)
{
  std::string status_name = getTransformStatusName(caller_id);
  std::string status_text = discoverFailureReason(frame_id, stamp, caller_id, reason);

  display->setStatusStd(StatusProperty::Error, status_name, status_text);
}
#endif

const std::shared_ptr<tf2_ros::Buffer> & FrameManager::getTFBufferPtr()
{
  return buffer_;
}

}  // namespace rviz_common
