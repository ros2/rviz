/*
 * Copyright (c) 2012, Willow Garage, Inc.
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


#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__POSE__POSE_DISPLAY_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__POSE__POSE_DISPLAY_HPP_

#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "rviz_common/message_filter_display.hpp"
#include "rviz_common/interaction/forwards.hpp"

#include "rviz_default_plugins/visibility_control.hpp"

namespace rviz_rendering
{
class Arrow;
class Axes;
class Shape;
}  // namespace rviz_rendering

namespace rviz_common
{
namespace properties
{
class ColorProperty;
class EnumProperty;
class FloatProperty;
}  // namespace properties
}  // namespace rviz_common

namespace rviz_default_plugins
{
namespace displays
{

class PoseDisplaySelectionHandler;

typedef std::shared_ptr<PoseDisplaySelectionHandler> PoseDisplaySelectionHandlerPtr;

/** @brief Accumulates and displays the pose from a geometry_msgs::PoseStamped message. */
class RVIZ_DEFAULT_PLUGINS_PUBLIC PoseDisplay : public
  rviz_common::MessageFilterDisplay<geometry_msgs::msg::PoseStamped>
{
  Q_OBJECT

public:
  enum Shape
  {
    Arrow,
    Axes,
  };

  PoseDisplay();

  ~PoseDisplay() override;
  void onInitialize() override;
  void reset() override;

protected:
  /** @brief Overridden from MessageFilterDisplay to get arrow/axes visibility correct. */
  void onEnable() override;
  void onDisable() override;
  void processMessage(geometry_msgs::msg::PoseStamped::ConstSharedPtr message) override;

private Q_SLOTS:
  void updateShapeVisibility();
  void updateColorAndAlpha();
  void updateShapeChoice();
  void updateAxisGeometry();
  void updateArrowGeometry();

private:
  void setupSelectionHandler();

  std::unique_ptr<rviz_rendering::Arrow> arrow_;
  std::unique_ptr<rviz_rendering::Axes> axes_;
  bool pose_valid_;
  PoseDisplaySelectionHandlerPtr coll_handler_;

  rviz_common::properties::EnumProperty * shape_property_;

  rviz_common::properties::ColorProperty * color_property_;
  rviz_common::properties::FloatProperty * alpha_property_;

  rviz_common::properties::FloatProperty * head_radius_property_;
  rviz_common::properties::FloatProperty * head_length_property_;
  rviz_common::properties::FloatProperty * shaft_radius_property_;
  rviz_common::properties::FloatProperty * shaft_length_property_;

  rviz_common::properties::FloatProperty * axes_length_property_;
  rviz_common::properties::FloatProperty * axes_radius_property_;

  friend class PoseDisplaySelectionHandler;
};

}  // namespace displays
}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__POSE__POSE_DISPLAY_HPP_
