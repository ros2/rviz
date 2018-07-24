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
#include "rviz_common/transformation/ros_helpers/ros_conversion_helpers.hpp"

namespace rviz_common
{

FrameTransformerTF::FrameTransformerTF()
: tf_wrapper_(std::make_shared<TFWrapper>(std::make_shared<tf2_ros::Buffer>(), true))
{}

FrameTransformerTF::FrameTransformerTF(std::shared_ptr<TFWrapper> wrapper)
: tf_wrapper_(wrapper)
{}

transformation::PoseStamped FrameTransformerTF::transform(
  const transformation::PoseStamped & pose_in, const std::string & target_frame)
{
  geometry_msgs::msg::PoseStamped out_pose;
  geometry_msgs::msg::PoseStamped in_pose = transformation::ros_helpers::toRosPoseStamped(pose_in);
  try {
    tf_wrapper_->transform(in_pose, out_pose, target_frame);
    return transformation::PoseStamped(out_pose);
  } catch (const tf2::LookupException & exception) {
    throw rviz_common::transformation::FrameTransformerException(exception.what());
  } catch (const tf2::ConnectivityException & exception) {
    throw rviz_common::transformation::FrameTransformerException(exception.what());
  } catch (const tf2::ExtrapolationException & exception) {
    throw rviz_common::transformation::FrameTransformerException(exception.what());
  } catch (const tf2::InvalidArgumentException & exception) {
    throw rviz_common::transformation::FrameTransformerException(exception.what());
  }
}

bool FrameTransformerTF::transformIsAvailable(
  const std::string & target_frame, const std::string & source_frame)
{
  try {
    tf_wrapper_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
    return true;
  } catch (const tf2::LookupException & exception) {
    (void) exception;
    return false;
  } catch (const tf2::ConnectivityException & exception) {
    (void) exception;
    return false;
  } catch (const tf2::ExtrapolationException & exception) {
    (void) exception;
    return false;
  } catch (const tf2::InvalidArgumentException & exception) {
    (void) exception;
    return false;
  }
}

bool FrameTransformerTF::transformHasProblems(
  const std::string & source_frame,
  const std::string & target_frame,
  const transformation::Time & time,
  std::string & error)
{
  std::string tf_error;
  tf2::TimePoint tf2_time(std::chrono::nanoseconds(time.nanoseconds_));
  bool transform_succeeded = tf_wrapper_->canTransform(
    target_frame, source_frame, tf2_time, tf_error);
  if (transform_succeeded) {
    return false;
  }

  bool fixed_frame_ok = !frameHasProblems(target_frame, error);
  bool ok = fixed_frame_ok && !frameHasProblems(source_frame, error);

  if (ok) {
    error = "No transform to fixed frame [" + target_frame + "].  TF error: [" + tf_error + "]";
    return true;
  }

  error = fixed_frame_ok ?
    "For frame [" + source_frame + "]: " + error :
    "For frame [" + source_frame + "]: Fixed " + error;

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

transformation::InternalFrameTransformerPtr FrameTransformerTF::getInternals()
{
  return tf_wrapper_;
}

void FrameTransformerTF::initialize(
  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node)
{
  tf_wrapper_->setListener(std::make_shared<tf2_ros::TransformListener>(
      *tf_wrapper_->getBuffer(), rviz_ros_node.lock()->get_raw_node(), false));
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
