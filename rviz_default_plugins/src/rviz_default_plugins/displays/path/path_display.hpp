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


#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__PATH_DISPLAY_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__PATH_DISPLAY_HPP_

#include "nav_msgs/msg/path.hpp"

#include "rviz_common/ros_topic_display.hpp"
#include "rviz_rendering/objects/arrow.hpp"
#include "rviz_rendering/objects/axes.hpp"
#include "rviz_rendering/objects/billboard_line.hpp"

namespace Ogre
{
class ManualObject;
}

namespace rviz_common
{
namespace properties
{
class ColorProperty;
class FloatProperty;
class IntProperty;
class EnumProperty;
class VectorProperty;
}
}

namespace rviz_default_plugins
{
namespace displays
{
/**
 * \class PathDisplay
 * \brief Displays a nav_msgs::msg::Path message
 */
class PathDisplay : public rviz_common::RosTopicDisplay<nav_msgs::msg::Path>
{
  Q_OBJECT

public:
  PathDisplay();
  virtual ~PathDisplay();

  /** @brief Overridden from Display. */
  virtual void reset();

protected:
  /** @brief Overridden from Display. */
  virtual void onInitialize();

  /** @brief Overridden from MessageFilterDisplay. */
  void processMessage(nav_msgs::msg::Path::ConstSharedPtr msg);

private Q_SLOTS:
  void updateBufferLength();
  void updateStyle();
  void updateLineWidth();
  void updateOffset();
  void updatePoseStyle();
  void updatePoseAxisGeometry();
  void updatePoseArrowColor();
  void updatePoseArrowGeometry();

private:
  void destroyObjects();
  void allocateArrowVector(std::vector<rviz_rendering::Arrow *> & arrow_vect, size_t num);
  void allocateAxesVector(std::vector<rviz_rendering::Axes *> & axes_vect, size_t num);
  void destroyPoseAxesChain();
  void destroyPoseArrowChain();

  std::vector<Ogre::ManualObject *> manual_objects_;
  std::vector<rviz_rendering::BillboardLine *> billboard_lines_;
  std::vector<std::vector<rviz_rendering::Axes *>> axes_chain_;
  std::vector<std::vector<rviz_rendering::Arrow *>> arrow_chain_;

  rviz_common::properties::EnumProperty * style_property_;
  rviz_common::properties::ColorProperty * color_property_;
  rviz_common::properties::FloatProperty * alpha_property_;
  rviz_common::properties::FloatProperty * line_width_property_;
  rviz_common::properties::IntProperty * buffer_length_property_;
  rviz_common::properties::VectorProperty * offset_property_;

  enum LineStyle
  {
    LINES,
    BILLBOARDS
  };

  // pose marker property
  rviz_common::properties::EnumProperty * pose_style_property_;
  rviz_common::properties::FloatProperty * pose_axes_length_property_;
  rviz_common::properties::FloatProperty * pose_axes_radius_property_;
  rviz_common::properties::ColorProperty * pose_arrow_color_property_;
  rviz_common::properties::FloatProperty * pose_arrow_shaft_length_property_;
  rviz_common::properties::FloatProperty * pose_arrow_head_length_property_;
  rviz_common::properties::FloatProperty * pose_arrow_shaft_diameter_property_;
  rviz_common::properties::FloatProperty * pose_arrow_head_diameter_property_;

  enum PoseStyle
  {
    NONE,
    AXES,
    ARROWS,
  };

};

} // namespace displays
} // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__PATH_DISPLAY_HPP_
