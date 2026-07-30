// Copyright (c) 2015, JSK Lab
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// This file is a ROS 2 port of jsk_rviz_plugins TFTrajectoryDisplay.

#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__TF_TRAJECTORY__TF_TRAJECTORY_DISPLAY_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__TF_TRAJECTORY__TF_TRAJECTORY_DISPLAY_HPP_

#include <chrono>
#include <deque>
#include <memory>
#include <string>

#include "rclcpp/time.hpp"
#include "rviz_common/display.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

namespace rviz_common
{
namespace properties
{
class ColorProperty;
class FloatProperty;
class TfFrameProperty;
}  // namespace properties
}  // namespace rviz_common

namespace rviz_rendering
{
class BillboardLine;
}  // namespace rviz_rendering

namespace rviz_default_plugins
{
namespace displays
{

class RVIZ_DEFAULT_PLUGINS_PUBLIC TFTrajectoryDisplay : public rviz_common::Display
{
  Q_OBJECT

public:
  /// Constructor for testing, which skips onInitialize().
  explicit TFTrajectoryDisplay(rviz_common::DisplayContext * context);
  TFTrajectoryDisplay();
  ~TFTrajectoryDisplay() override;

  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;
  void fixedFrameChanged() override;
  void reset() override;
  void update(std::chrono::nanoseconds wall_dt, std::chrono::nanoseconds ros_dt) override;

private Q_SLOTS:
  void updateFrame();
  void updateDuration();
  void updateColor();
  void updateLineWidth();

private:
  struct TrajectoryPoint
  {
    rclcpp::Time stamp;
    float x;
    float y;
    float z;
  };

  void clearTrajectory();
  void trimExpiredPoints(const rclcpp::Time & now);
  void redrawTrajectory();

  rviz_common::properties::TfFrameProperty * frame_property_;
  rviz_common::properties::FloatProperty * duration_property_;
  rviz_common::properties::FloatProperty * line_width_property_;
  rviz_common::properties::ColorProperty * color_property_;

  std::unique_ptr<rviz_rendering::BillboardLine> trajectory_line_;
  std::deque<TrajectoryPoint> trajectory_;
  std::string frame_id_;
};

}  // namespace displays
}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__TF_TRAJECTORY__TF_TRAJECTORY_DISPLAY_HPP_
