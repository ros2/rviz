/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__POSE_ARRAY__POSE_ARRAY_DISPLAY_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__POSE_ARRAY__POSE_ARRAY_DISPLAY_HPP_

#include <memory>
#include <vector>

#include "geometry_msgs/msg/pose_array.hpp"

#include "rviz_rendering/objects/shape.hpp"
#include "rviz_common/message_filter_display.hpp"

#include "rviz_default_plugins/visibility_control.hpp"

// TODO(botteroa): Originally the display extended the MessageFilterDisplay. Revisit when available.
// #include "rviz_common/message_filter_display.hpp"

namespace Ogre
{
class ManualObject;
}  // namespace Ogre

namespace rviz_common
{
namespace properties
{
class EnumProperty;
class ColorProperty;
class FloatProperty;
}  // namespace properties
}  // namespace rviz_common

namespace rviz_rendering
{
class Arrow;
class Axes;
}  // namespace rviz_rendering

namespace rviz_default_plugins
{
namespace displays
{
class FlatArrowsArray;
struct OgrePose
{
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
};

/** @brief Displays a geometry_msgs/PoseArray message as a bunch of line-drawn arrows. */
class RVIZ_DEFAULT_PLUGINS_PUBLIC PoseArrayDisplay : public
  rviz_common::MessageFilterDisplay<geometry_msgs::msg::PoseArray>
{
  Q_OBJECT

public:
  // TODO(botteroa-si): Constructor for testing, remove once ros_nodes can be mocked and call
  // initialize instead
  PoseArrayDisplay(
    rviz_common::DisplayContext * display_context,
    Ogre::SceneNode * scene_node);
  PoseArrayDisplay();
  ~PoseArrayDisplay() override;

  void processMessage(geometry_msgs::msg::PoseArray::ConstSharedPtr msg) override;
  void setShape(QString shape);  // for testing

protected:
  void onInitialize() override;
  void reset() override;

private Q_SLOTS:
  /// Update the interface and visible shapes based on the selected shape type.
  void updateShapeChoice();

  /// Update the arrow color.
  void updateArrowColor();

  /// Update the flat arrow geometry.
  void updateArrow2dGeometry();

  /// Update the 3D arrow geometry.
  void updateArrow3dGeometry();

  /// Update the axes geometry.
  void updateAxesGeometry();

private:
  void initializeProperties();
  bool validateFloats(const geometry_msgs::msg::PoseArray & msg);
  bool setTransform(std_msgs::msg::Header const & header);
  void updateDisplay();
  void updateArrows2d();
  void updateArrows3d();
  void updateAxes();
  std::unique_ptr<rviz_rendering::Axes> makeAxes();
  std::unique_ptr<rviz_rendering::Arrow> makeArrow3d();

  std::vector<OgrePose> poses_;
  std::unique_ptr<FlatArrowsArray> arrows2d_;
  std::vector<std::unique_ptr<rviz_rendering::Arrow>> arrows3d_;
  std::vector<std::unique_ptr<rviz_rendering::Axes>> axes_;

  Ogre::SceneNode * arrow_node_;
  Ogre::SceneNode * axes_node_;

  rviz_common::properties::EnumProperty * shape_property_;
  rviz_common::properties::ColorProperty * arrow_color_property_;
  rviz_common::properties::FloatProperty * arrow_alpha_property_;

  rviz_common::properties::FloatProperty * arrow2d_length_property_;

  rviz_common::properties::FloatProperty * arrow3d_head_radius_property_;
  rviz_common::properties::FloatProperty * arrow3d_head_length_property_;
  rviz_common::properties::FloatProperty * arrow3d_shaft_radius_property_;
  rviz_common::properties::FloatProperty * arrow3d_shaft_length_property_;

  rviz_common::properties::FloatProperty * axes_length_property_;
  rviz_common::properties::FloatProperty * axes_radius_property_;
};

}  // namespace displays
}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__POSE_ARRAY__POSE_ARRAY_DISPLAY_HPP_
