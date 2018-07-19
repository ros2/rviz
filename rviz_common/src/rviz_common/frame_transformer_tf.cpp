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

#include "frame_transformer_tf.hpp"

#include <memory>
#include <string>
#include <vector>

#include "rviz_common/logging.hpp"

namespace rviz_common
{

FrameTransformerTF::FrameTransformerTF()
{
  wrapper_ = std::make_shared<TFWrapper>();
  wrapper_->buffer_ = std::make_shared<tf2_ros::Buffer>();
  wrapper_->buffer_->setUsingDedicatedThread(true);
}

FrameTransformerTF::FrameTransformerTF(std::shared_ptr<TFWrapper> wrapper)
: wrapper_(wrapper)
{}

bool
FrameTransformerTF::transform(
  const geometry_msgs::msg::PoseStamped & pose_in,
  geometry_msgs::msg::PoseStamped & pose_out,
  const std::string & frame)
{
  try {
    wrapper_->buffer_->transform(pose_in, pose_out, frame);
  } catch (const tf2::LookupException & exception) {
    RVIZ_COMMON_LOG_ERROR_STREAM(exception.what());
    return false;
  } catch (const tf2::ConnectivityException & exception) {
    RVIZ_COMMON_LOG_ERROR_STREAM(exception.what());
    return false;
  } catch (const tf2::ExtrapolationException & exception) {
    RVIZ_COMMON_LOG_ERROR_STREAM(exception.what());
    return false;
  } catch (const tf2::InvalidArgumentException & exception) {
    RVIZ_COMMON_LOG_ERROR_STREAM(exception.what());
    return false;
  }
  return true;
}

bool
FrameTransformerTF::lastAvailableTransform(
  const std::string & target_frame,
  const std::string & source_frame,
  geometry_msgs::msg::TransformStamped & transform)
{
  try {
    transform = wrapper_->buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
  } catch (const tf2::LookupException & exception) {
    RVIZ_COMMON_LOG_ERROR_STREAM(exception.what());
    return false;
  } catch (const tf2::ConnectivityException & exception) {
    RVIZ_COMMON_LOG_ERROR_STREAM(exception.what());
    return false;
  } catch (const tf2::ExtrapolationException & exception) {
    RVIZ_COMMON_LOG_ERROR_STREAM(exception.what());
    return false;
  } catch (const tf2::InvalidArgumentException & exception) {
    RVIZ_COMMON_LOG_ERROR_STREAM(exception.what());
    return false;
  }
  return true;
}

bool
FrameTransformerTF::transformHasProblems(
  const std::string & frame,
  const std::string & fixed_frame,
  const rclcpp::Time & time,
  std::string & error)
{
  std::string tf_error;
  tf2::TimePoint tf2_time(std::chrono::nanoseconds(time.nanoseconds()));
  bool transform_succeeded = wrapper_->buffer_->canTransform(
    fixed_frame, frame, tf2_time, &tf_error);
  if (transform_succeeded) {
    return false;
  }

  bool fixed_frame_ok = !frameHasProblems(fixed_frame, error);
  bool ok = fixed_frame_ok && !frameHasProblems(frame, error);

  if (ok) {
    error = "No transform to fixed frame [" + fixed_frame + "].  TF error: [" + tf_error + "]";
    return true;
  }

  error = fixed_frame_ok ?
    "For frame [" + frame + "]: " + error :
    "For frame [" + frame + "]: Fixed " + error;

  return true;
}

bool
FrameTransformerTF::frameHasProblems(const std::string & frame, std::string & error)
{
  if (!wrapper_->buffer_->_frameExists(frame)) {
    error = "Frame [" + frame + "] does not exist";
    return true;
  }

  return false;
}

void FrameTransformerTF::initialize(
  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node)
{
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
    *wrapper_->buffer_, rviz_ros_node.lock()->get_raw_node(), false);
}

void FrameTransformerTF::clear()
{
  wrapper_->buffer_->clear();
}

std::vector<std::string> FrameTransformerTF::getAllFrameNames()
{
  std::vector<std::string> frames;
  wrapper_->buffer_->_getFrameStrings(frames);
  return frames;
}

}  // namespace rviz_common
