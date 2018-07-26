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

#ifndef RVIZ_DEFAULT_PLUGINS__TRANSFORMATION__TF_WRAPPER_HPP_
#define RVIZ_DEFAULT_PLUGINS__TRANSFORMATION__TF_WRAPPER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "rviz_common/transformation/frame_transformer.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

namespace rviz_default_plugins
{
namespace transformation
{
class TFWrapper
  : public rviz_common::transformation::InternalFrameTransformer
{
public:
  RVIZ_DEFAULT_PLUGINS_PUBLIC
  TFWrapper(std::shared_ptr<tf2_ros::Buffer> buffer, bool using_dedicated_thread);
  RVIZ_DEFAULT_PLUGINS_PUBLIC
  ~TFWrapper() override = default;

  RVIZ_DEFAULT_PLUGINS_PUBLIC
  void transform(
    const geometry_msgs::msg::PoseStamped & pose_in,
    geometry_msgs::msg::PoseStamped & pose_out,
    const std::string & frame);

  RVIZ_DEFAULT_PLUGINS_PUBLIC
  geometry_msgs::msg::TransformStamped lookupTransform(
    const std::string & target_frame,
    const std::string & source_frame,
    const tf2::TimePoint & time);

  RVIZ_DEFAULT_PLUGINS_PUBLIC
  bool canTransform(
    const std::string & fixed_frame,
    const std::string & frame,
    tf2::TimePoint time,
    std::string & error);

  RVIZ_DEFAULT_PLUGINS_PUBLIC
  std::vector<std::string> getFrameStrings();
  RVIZ_DEFAULT_PLUGINS_PUBLIC
  std::shared_ptr<tf2_ros::Buffer> getBuffer();

  RVIZ_DEFAULT_PLUGINS_PUBLIC
  bool frameExists(const std::string & frame);

  RVIZ_DEFAULT_PLUGINS_PUBLIC
  void setListener(std::shared_ptr<tf2_ros::TransformListener> tf_listener);

  RVIZ_DEFAULT_PLUGINS_PUBLIC
  void clear();

private:
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace transformation
}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__TRANSFORMATION__TF_WRAPPER_HPP_