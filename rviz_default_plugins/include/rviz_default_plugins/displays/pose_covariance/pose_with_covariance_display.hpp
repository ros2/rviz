/*
 * Copyright (c) 2017, Ellon Paiva Mendes @ LAAS-CNRS
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

#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__POSE_COVARIANCE__POSE_WITH_COVARIANCE_DISPLAY_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__POSE_COVARIANCE__POSE_WITH_COVARIANCE_DISPLAY_HPP_

#include <memory>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include "rviz_common/message_filter_display.hpp"
#include "rviz_common/interaction/forwards.hpp"

namespace rviz_rendering
{
class Arrow;
class Axes;
class Shape;
class CovarianceVisual;
}  // namespace rviz_rendering

namespace rviz_common
{
namespace properties
{
class ColorProperty;
class EnumProperty;
class FloatProperty;
class BoolProperty;
class CovarianceProperty;
}  // namespace properties
}  // namespace rviz_common

namespace rviz_default_plugins
{

namespace displays
{

class PoseWithCovSelectionHandler;
typedef std::shared_ptr<PoseWithCovSelectionHandler>
  PoseWithCovarianceDisplaySelectionHandlerPtr;

/** @brief Displays the pose from a geometry_msgs::msg::PoseWithCovarianceStamped message. */
class PoseWithCovarianceDisplay
  : public rviz_common::MessageFilterDisplay<geometry_msgs::msg::PoseWithCovarianceStamped>
{
  Q_OBJECT

public:
  enum Shape
  {
    Arrow,
    Axes,
  };

  PoseWithCovarianceDisplay();
  ~PoseWithCovarianceDisplay() override;

  void onInitialize() override;
  void reset() override;

protected:
  /** @brief Overridden from MessageFilterDisplay to get Arrow/Axes visibility correct. */
  void onEnable() override;
  void
  processMessage(geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr message) override;

private Q_SLOTS:
  void updateShapeVisibility();
  void updateColorAndAlpha();
  void updateShapeChoice();
  void updateAxisGeometry();
  void updateArrowGeometry();
  void updateCovariance();

private:
  void setupSelectionHandler();

  std::shared_ptr<rviz_rendering::Arrow> arrow_;
  std::shared_ptr<rviz_rendering::Axes> axes_;
  std::shared_ptr<rviz_rendering::CovarianceVisual> covariance_;
  bool pose_valid_;
  PoseWithCovarianceDisplaySelectionHandlerPtr coll_handler_;

  rviz_common::properties::EnumProperty * shape_property_;

  rviz_common::properties::ColorProperty * color_property_;
  rviz_common::properties::FloatProperty * alpha_property_;

  rviz_common::properties::FloatProperty * head_radius_property_;
  rviz_common::properties::FloatProperty * head_length_property_;
  rviz_common::properties::FloatProperty * shaft_radius_property_;
  rviz_common::properties::FloatProperty * shaft_length_property_;

  rviz_common::properties::FloatProperty * axes_length_property_;
  rviz_common::properties::FloatProperty * axes_radius_property_;

  rviz_common::properties::CovarianceProperty * covariance_property_;

  friend class PoseWithCovSelectionHandler;
};

}  // namespace displays
}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__POSE_COVARIANCE__POSE_WITH_COVARIANCE_DISPLAY_HPP_
