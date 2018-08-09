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

#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__RANGE__RANGE_DISPLAY_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__RANGE__RANGE_DISPLAY_HPP_

#include <memory>
#include <vector>

#include "sensor_msgs/msg/range.hpp"

#include "rviz_common/ros_topic_display.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

namespace rviz_rendering
{
class Shape;
}  // namespace rviz_rendering

namespace rviz_common
{
class QueueSizeProperty;
namespace properties
{
class ColorProperty;
class FloatProperty;
class IntProperty;
}  // namespace properties
}  // namespace rviz_common


namespace rviz_default_plugins
{
namespace displays
{
/**
 * \class RangeDisplay
 * \brief Displays a sensor_msgs::Range message as a cone.
 */
class RVIZ_DEFAULT_PLUGINS_PUBLIC RangeDisplay : public
  rviz_common::RosTopicDisplay<sensor_msgs::msg::Range>
{
  Q_OBJECT

public:
  // TODO(botteroa-si): Constructor for testing. Remove once ros nodes can be mocked and
  // initialize() can be called
  explicit RangeDisplay(rviz_common::DisplayContext * display_context);

  RangeDisplay();

  ~RangeDisplay() override;

  void reset() override;

  void processMessage(sensor_msgs::msg::Range::ConstSharedPtr msg) override;

protected:
  void onInitialize() override;

private Q_SLOTS:
  void updateBufferLength();
  void updateColorAndAlpha();

private:
  float getDisplayedRange(sensor_msgs::msg::Range::ConstSharedPtr msg);
  geometry_msgs::msg::Pose getPose(float displayed_range);

  std::vector<std::shared_ptr<rviz_rendering::Shape>> cones_;

  rviz_common::properties::ColorProperty * color_property_;
  rviz_common::properties::FloatProperty * alpha_property_;
  rviz_common::properties::IntProperty * buffer_length_property_;
  std::unique_ptr<rviz_common::QueueSizeProperty> queue_size_property_;
};

}  // namespace displays
}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__RANGE__RANGE_DISPLAY_HPP_
