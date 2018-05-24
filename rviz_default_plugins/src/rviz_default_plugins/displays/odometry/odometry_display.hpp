/*
 * Copyright (c) 2008, Willow Garage, Inc.
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


#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__ODOMETRY__ODOMETRY_DISPLAY_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__ODOMETRY__ODOMETRY_DISPLAY_HPP_

#include <deque>

#ifndef Q_MOC_RUN

// TODO(Martin-Idel-SI): Reenable once available
// #include <tf/message_filter.h>
#include "nav_msgs/msg/odometry.hpp"
#endif

#include "rviz_common/ros_topic_display.hpp"

namespace rviz_rendering
{
class Arrow;
class Axes;
}  // namespace rviz_rendering

namespace rviz_common
{
namespace properties
{
class ColorProperty;
class FloatProperty;
class IntProperty;
class EnumProperty;
class CovarianceProperty;
}  // namespace properties
}  // namespace rviz_common

namespace rviz_default_plugins
{
namespace displays
{
/**
 * \class OdometryDisplay
 * \brief Accumulates and displays the pose from a nav_msgs::Odometry message
 */
class OdometryDisplay : public rviz_common::RosTopicDisplay<nav_msgs::msg::Odometry>
{
  Q_OBJECT

public:
  enum Shape
  {
    ArrowShape,
    AxesShape,
  };

  OdometryDisplay();

  virtual ~OdometryDisplay();

  // Overides of MessageFilterDisplay
  virtual void onInitialize();

  virtual void reset();

  // Overides of Display
  virtual void update(float wall_dt, float ros_dt);

protected:
  /** @brief Overridden from MessageFilterDisplay to get Arrow/Axes visibility correct. */
  virtual void onEnable();

private
  Q_SLOTS:
  void

  updateShapeChoice();

  void updateShapeVisibility();

  void updateColorAndAlpha();

  void updateArrowsGeometry();

  void updateAxisGeometry();

private:
  void updateGeometry(rviz_rendering::Arrow * arrow);

  void updateGeometry(rviz_rendering::Axes * axes);

  void clear();

  virtual void processMessage(nav_msgs::msg::Odometry::ConstSharedPtr message);

  typedef std::deque<rviz_rendering::Arrow *> D_Arrow;
  typedef std::deque<rviz_rendering::Axes *> D_Axes;

  D_Arrow arrows_;
  D_Axes axes_;

  nav_msgs::msg::Odometry::ConstSharedPtr last_used_message_;

  rviz_common::properties::EnumProperty * shape_property_;

  rviz_common::properties::ColorProperty * color_property_;
  rviz_common::properties::FloatProperty * alpha_property_;
  rviz_common::properties::FloatProperty * position_tolerance_property_;
  rviz_common::properties::FloatProperty * angle_tolerance_property_;
  rviz_common::properties::IntProperty * keep_property_;

  rviz_common::properties::FloatProperty * head_radius_property_;
  rviz_common::properties::FloatProperty * head_length_property_;
  rviz_common::properties::FloatProperty * shaft_radius_property_;
  rviz_common::properties::FloatProperty * shaft_length_property_;

  rviz_common::properties::FloatProperty * axes_length_property_;
  rviz_common::properties::FloatProperty * axes_radius_property_;

  rviz_common::properties::CovarianceProperty * covariance_property_;
};

}  // namespace displays
}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__ODOMETRY__ODOMETRY_DISPLAY_HPP_
