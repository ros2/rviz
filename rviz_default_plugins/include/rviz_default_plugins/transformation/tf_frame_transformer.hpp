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

#ifndef RVIZ_DEFAULT_PLUGINS__TRANSFORMATION__TF_FRAME_TRANSFORMER_HPP_
#define RVIZ_DEFAULT_PLUGINS__TRANSFORMATION__TF_FRAME_TRANSFORMER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <OgreVector.h>
#include <OgreQuaternion.h>

#include "rviz_common/transformation/frame_transformer.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction.hpp"
#include "rviz_default_plugins/transformation/tf_wrapper.hpp"

#include "rviz_default_plugins/visibility_control.hpp"

namespace rviz_default_plugins
{
namespace transformation
{
class TFFrameTransformer : public rviz_common::transformation::FrameTransformer
{
public:
  RVIZ_DEFAULT_PLUGINS_PUBLIC
  TFFrameTransformer();

  RVIZ_DEFAULT_PLUGINS_PUBLIC
  explicit TFFrameTransformer(std::shared_ptr<TFWrapper> wrapper);

  ~TFFrameTransformer() override = default;

  RVIZ_DEFAULT_PLUGINS_PUBLIC
  void
  initialize(
    rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node,
    rclcpp::Clock::SharedPtr clock) override;

  RVIZ_DEFAULT_PLUGINS_PUBLIC
  void
  clear() override;

  RVIZ_DEFAULT_PLUGINS_PUBLIC
  std::vector<std::string>
  getAllFrameNames() const override;

  RVIZ_DEFAULT_PLUGINS_PUBLIC
  geometry_msgs::msg::PoseStamped
  transform(
    const geometry_msgs::msg::PoseStamped & pose_in,
    const std::string & target_frame) override;

  RVIZ_DEFAULT_PLUGINS_PUBLIC
  bool
  frameHasProblems(const std::string & frame, std::string & error) const override;

  RVIZ_DEFAULT_PLUGINS_PUBLIC
  rviz_common::transformation::TransformationLibraryConnector::WeakPtr
  getConnector() override;

  geometry_msgs::msg::TransformStamped
  lookupTransform(
    const std::string & target_frame,
    const std::string & source_frame,
    const tf2::TimePoint & time) const override;

  geometry_msgs::msg::TransformStamped
  lookupTransform(
    const std::string & target_frame,
    const tf2::TimePoint & target_time,
    const std::string & source_frame,
    const tf2::TimePoint & source_time,
    const std::string & fixed_frame) const override;

  bool
  canTransform(
    const std::string & target_frame,
    const std::string & source_frame,
    const tf2::TimePoint & time,
    std::string * error_msg) const override;

  bool
  canTransform(
    const std::string & target_frame,
    const tf2::TimePoint & target_time,
    const std::string & source_frame,
    const tf2::TimePoint & source_time,
    const std::string & fixed_frame,
    std::string * error_msg) const override;

  tf2_ros::TransformStampedFuture
  waitForTransform(
    const std::string & target_frame,
    const std::string & source_frame,
    const tf2::TimePoint & time,
    const tf2::Duration & timeout,
    tf2_ros::TransformReadyCallback callback) override;

  void
  cancel(
    const tf2_ros::TransformStampedFuture & ts_future) override;

private:
  std::shared_ptr<TFWrapper> tf_wrapper_;
};
}  // namespace transformation
}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__TRANSFORMATION__TF_FRAME_TRANSFORMER_HPP_
