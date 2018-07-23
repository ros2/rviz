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
: tf_wrapper_(std::make_shared<TFWrapper>(std::make_shared<tf2_ros::Buffer>(), true)),
  type_id_("tf_transformer")
{}

FrameTransformerTF::FrameTransformerTF(std::shared_ptr<TFWrapper> wrapper)
: tf_wrapper_(wrapper), type_id_("tf_transformer")
{}

bool FrameTransformerTF::transform(
  const geometry_msgs::msg::PoseStamped & pose_in,
  geometry_msgs::msg::PoseStamped & pose_out,
  const std::string & frame)
{
  try {
    tf_wrapper_->transform(pose_in, pose_out, frame);
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

bool FrameTransformerTF::lastAvailableTransform(
  const std::string & target_frame,
  const std::string & source_frame,
  geometry_msgs::msg::TransformStamped & transform)
{
  try {
    transform = tf_wrapper_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
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

bool FrameTransformerTF::transformHasProblems(
  const std::string & frame,
  const std::string & fixed_frame,
  const rclcpp::Time & time,
  std::string & error)
{
  std::string tf_error;
  tf2::TimePoint tf2_time(std::chrono::nanoseconds(time.nanoseconds()));
  bool transform_succeeded = tf_wrapper_->canTransform(
    fixed_frame, frame, tf2_time, tf_error);
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

bool FrameTransformerTF::frameHasProblems(const std::string & frame, std::string & error)
{
  if (!tf_wrapper_->frameExists(frame)) {
    error = "Frame [" + frame + "] does not exist";
    return true;
  }

  return false;
}

InternalFrameTransformerPtr FrameTransformerTF::getInternals()
{
  return tf_wrapper_;
}

std::string FrameTransformerTF::getTypeId()
{
  return type_id_;
}

void FrameTransformerTF::initialize(
  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node)
{
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
    *tf_wrapper_->getBuffer(), rviz_ros_node.lock()->get_raw_node(), false);
}

void FrameTransformerTF::clear()
{
  tf_wrapper_->clear();
}

std::vector<std::string> FrameTransformerTF::getAllFrameNames()
{
  return tf_wrapper_->getFrameStrings();
}

}  // namespace rviz_common
