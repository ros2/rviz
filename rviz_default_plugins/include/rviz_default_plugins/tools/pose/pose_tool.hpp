/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

#ifndef RVIZ_DEFAULT_PLUGINS__TOOLS__POSE__POSE_TOOL_HPP_
#define RVIZ_DEFAULT_PLUGINS__TOOLS__POSE__POSE_TOOL_HPP_

#include <memory>
#include <string>
#include <utility>

#include <OgreVector.h>

#include <QCursor>  // NOLINT cpplint cannot handle include order here

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "rviz_common/tool.hpp"
#include "rviz_rendering/viewport_projection_finder.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

namespace rviz_rendering
{
class Arrow;
}  // namespace rviz_rendering

namespace rviz_default_plugins
{
namespace tools
{
class RVIZ_DEFAULT_PLUGINS_PUBLIC PoseTool : public rviz_common::Tool
{
public:
  PoseTool();

  ~PoseTool() override;

  void onInitialize() override;

  void activate() override;

  void deactivate() override;

  int processMouseEvent(rviz_common::ViewportMouseEvent & event) override;

protected:
  virtual void onPoseSet(double x, double y, double theta) = 0;

  geometry_msgs::msg::Quaternion orientationAroundZAxis(double angle);

  void logPose(
    std::string designation,
    geometry_msgs::msg::Point position,
    geometry_msgs::msg::Quaternion orientation,
    double angle,
    std::string frame);

  std::shared_ptr<rviz_rendering::Arrow> arrow_;

  enum State
  {
    Position,
    Orientation
  };
  State state_;
  double angle_;

  Ogre::Vector3 arrow_position_;
  std::shared_ptr<rviz_rendering::ViewportProjectionFinder> projection_finder_;

private:
  int processMouseLeftButtonPressed(std::pair<bool, Ogre::Vector3> xy_plane_intersection);
  int processMouseMoved(std::pair<bool, Ogre::Vector3> xy_plane_intersection);
  int processMouseLeftButtonReleased();
  void makeArrowVisibleAndSetOrientation(double angle);
  double calculateAngle(Ogre::Vector3 start_point, Ogre::Vector3 end_point);
};

}  // namespace tools
}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__TOOLS__POSE__POSE_TOOL_HPP_
