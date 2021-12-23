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

#include "rviz_default_plugins/transformation/tf_frame_transformer.hpp"

#include <memory>
#include <string>
#include <vector>

#include "rviz_common/logging.hpp"
#include "rviz_common/transformation/frame_transformer.hpp"
#include "rviz_common/transformation/tf2_helpers/tf2_conversion_helpers.hpp"

namespace rviz_default_plugins
{
namespace transformation
{

TFFrameTransformer::TFFrameTransformer()
: tf_wrapper_(std::make_shared<TFWrapper>())
{}

TFFrameTransformer::TFFrameTransformer(std::shared_ptr<TFWrapper> wrapper)
: tf_wrapper_(wrapper)
{}

geometry_msgs::msg::PoseStamped TFFrameTransformer::transform(
  const geometry_msgs::msg::PoseStamped & in_pose, const std::string & target_frame)
{
  geometry_msgs::msg::PoseStamped out_pose;
  try {
    tf_wrapper_->transform(in_pose, out_pose, target_frame);
    return out_pose;
  } catch (const tf2::LookupException & exception) {
    std::string prefix = "[tf2::LookupException]: ";
    throw rviz_common::transformation::FrameTransformerException(
            std::string(prefix + exception.what()).c_str());
  } catch (const tf2::ConnectivityException & exception) {
    std::string prefix = "[tf2::ConnectivityException]: ";
    throw rviz_common::transformation::FrameTransformerException(
            std::string(prefix + exception.what()).c_str());
  } catch (const tf2::ExtrapolationException & exception) {
    std::string prefix = "[tf2::ExtrapolationException]: ";
    throw rviz_common::transformation::FrameTransformerException(
            std::string(prefix + exception.what()).c_str());
  } catch (const tf2::InvalidArgumentException & exception) {
    std::string prefix = "[tf2::InvalidArgumentException]: ";
    throw rviz_common::transformation::FrameTransformerException(
            std::string(prefix + exception.what()).c_str());
  }
}

geometry_msgs::msg::TransformStamped TFFrameTransformer::lookupTransform(
  const std::string & target_frame,
  const std::string & source_frame,
  const tf2::TimePoint & time) const
{
  try {
    return tf_wrapper_->lookupTransform(target_frame, source_frame, time);
  } catch (const tf2::LookupException & exception) {
    std::string prefix = "[tf2::LookupException]: ";
    throw rviz_common::transformation::FrameTransformerException(
            std::string(prefix + exception.what()).c_str());
  } catch (const tf2::ConnectivityException & exception) {
    std::string prefix = "[tf2::ConnectivityException]: ";
    throw rviz_common::transformation::FrameTransformerException(
            std::string(prefix + exception.what()).c_str());
  } catch (const tf2::ExtrapolationException & exception) {
    std::string prefix = "[tf2::ExtrapolationException]: ";
    throw rviz_common::transformation::FrameTransformerException(
            std::string(prefix + exception.what()).c_str());
  } catch (const tf2::InvalidArgumentException & exception) {
    std::string prefix = "[tf2::InvalidArgumentException]: ";
    throw rviz_common::transformation::FrameTransformerException(
            std::string(prefix + exception.what()).c_str());
  }
}

geometry_msgs::msg::TransformStamped TFFrameTransformer::lookupTransform(
  const std::string & target_frame,
  const tf2::TimePoint & target_time,
  const std::string & source_frame,
  const tf2::TimePoint & source_time,
  const std::string & fixed_frame) const
{
  try {
    return tf_wrapper_->lookupTransform(
      target_frame, target_time, source_frame, source_time, fixed_frame);
  } catch (const tf2::LookupException & exception) {
    std::string prefix = "[tf2::LookupException]: ";
    throw rviz_common::transformation::FrameTransformerException(
            std::string(prefix + exception.what()).c_str());
  } catch (const tf2::ConnectivityException & exception) {
    std::string prefix = "[tf2::ConnectivityException]: ";
    throw rviz_common::transformation::FrameTransformerException(
            std::string(prefix + exception.what()).c_str());
  } catch (const tf2::ExtrapolationException & exception) {
    std::string prefix = "[tf2::ExtrapolationException]: ";
    throw rviz_common::transformation::FrameTransformerException(
            std::string(prefix + exception.what()).c_str());
  } catch (const tf2::InvalidArgumentException & exception) {
    std::string prefix = "[tf2::InvalidArgumentException]: ";
    throw rviz_common::transformation::FrameTransformerException(
            std::string(prefix + exception.what()).c_str());
  }
}

bool TFFrameTransformer::canTransform(
  const std::string & target_frame,
  const std::string & source_frame,
  const tf2::TimePoint & time,
  std::string * error) const
{
  std::string tf2_error;
  bool transform_succeeded = tf_wrapper_->canTransform(
    target_frame, source_frame, time, tf2_error);
  if (transform_succeeded) {
    return true;
  }

  if (error) {
    bool target_frame_ok = !frameHasProblems(target_frame, *error);
    bool ok = target_frame_ok && !frameHasProblems(source_frame, *error);

    if (ok) {
      *error = "No transform to fixed frame [" + target_frame + "]. "
        "TF error: [" + tf2_error + "]";
      return false;
    }

    *error = target_frame_ok ?
      "For frame [" + source_frame + "]: " + *error :
      "For frame [" + source_frame + "]: Fixed " + *error;
  }
  return false;
}

bool TFFrameTransformer::canTransform(
  const std::string & target_frame,
  const tf2::TimePoint & target_time,
  const std::string & source_frame,
  const tf2::TimePoint & source_time,
  const std::string & fixed_frame,
  std::string * error) const
{
  std::string tf2_error;
  bool transform_succeeded = tf_wrapper_->canTransform(
    target_frame, target_time, source_frame, source_time, fixed_frame, tf2_error);
  if (transform_succeeded) {
    return true;
  }

  if (error) {
    bool target_frame_ok = !frameHasProblems(target_frame, *error);
    bool ok = target_frame_ok && !frameHasProblems(source_frame, *error);

    if (ok) {
      *error = "No transform to fixed frame [" + target_frame + "]. "
        "TF error: [" + tf2_error + "]";
      return false;
    }

    *error = target_frame_ok ?
      "For frame [" + source_frame + "]: " + *error :
      "For frame [" + source_frame + "]: Fixed " + *error;
  }

  return false;
}

tf2_ros::TransformStampedFuture TFFrameTransformer::waitForTransform(
  const std::string & target_frame,
  const std::string & source_frame,
  const tf2::TimePoint & time,
  const tf2::Duration & timeout,
  tf2_ros::TransformReadyCallback callback)
{
  return tf_wrapper_->waitForTransform(target_frame, source_frame, time, timeout, callback);
}

void TFFrameTransformer::cancel(
  const tf2_ros::TransformStampedFuture & ts_future)
{
  tf_wrapper_->cancel(ts_future);
}

bool TFFrameTransformer::frameHasProblems(const std::string & frame, std::string & error) const
{
  if (!tf_wrapper_->frameExists(frame)) {
    error = "Frame [" + frame + "] does not exist";
    return true;
  }

  return false;
}

rviz_common::transformation::TransformationLibraryConnector::WeakPtr
TFFrameTransformer::getConnector()
{
  return tf_wrapper_;
}

void TFFrameTransformer::initialize(
  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node,
  rclcpp::Clock::SharedPtr clock)
{
  tf_wrapper_->initialize(clock, rviz_ros_node, true);
}

void TFFrameTransformer::clear()
{
  tf_wrapper_->clear();
}

std::vector<std::string> TFFrameTransformer::getAllFrameNames() const
{
  return tf_wrapper_->getFrameStrings();
}

}  // namespace transformation
}  // namespace rviz_default_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  rviz_default_plugins::transformation::TFFrameTransformer,
  rviz_common::transformation::FrameTransformer)
