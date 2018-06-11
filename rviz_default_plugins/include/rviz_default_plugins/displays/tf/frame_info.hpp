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

#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__TF__FRAME_INFO_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__TF__FRAME_INFO_HPP_

#include <string>

#include "tf2/time.h"

#include "rviz_default_plugins/displays/tf/tf_display.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

namespace rviz_common
{

namespace properties
{
class Property;
class BoolProperty;
class StringProperty;
class VectorProperty;
class QuaternionProperty;
}  // namespace properties
}  // namespace rviz_common

namespace rviz_default_plugins
{
namespace displays
{

/** @brief Internal class needed only by TFDisplay. */
class RVIZ_DEFAULT_PLUGINS_PUBLIC FrameInfo : public QObject
{
  Q_OBJECT

public:
  explicit FrameInfo(TFDisplay * display);

  static const Ogre::ColourValue ARROW_HEAD_COLOR;
  static const Ogre::ColourValue ARROW_SHAFT_COLOR;

  /** @brief Set this frame to be visible or invisible. */
  void setEnabled(bool enabled);

  void updatePositionAndOrientation(
    const Ogre::Vector3 & position, const Ogre::Quaternion & orientation, float scale);

  void setVisible(bool show_frame);
  void setNamesVisible(bool show_names);
  void setAxesVisible(bool show_axes);
  void setParentArrowVisible(bool show_parent_arrow);
  void setLastUpdate(const tf2::TimePoint & latest_time);

  void updateTreeProperty(rviz_common::properties::Property * parent);
  void updateColorForAge(double age, double frame_timeout) const;
  void updateParentArrow(
    const Ogre::Vector3 & position,
    const Ogre::Vector3 & parent_position,
    float scale);

public Q_SLOTS:
  /** @brief Update whether the frame is visible or not, based on the enabled_property_
   * in this FrameInfo. */
  void updateVisibilityFromFrame();

  /** @brief Update whether the frame is visible or not, based on the enabled_property_
   * in the selection handler. */
  void updateVisibilityFromSelection();

public:
  TFDisplay * display_;
  std::string name_;
  std::string parent_;
  rviz_rendering::Axes * axes_;
  rviz_common::interaction::CollObjectHandle axes_coll_;
  FrameSelectionHandlerPtr selection_handler_;
  rviz_rendering::Arrow * parent_arrow_;
  rviz_rendering::MovableText * name_text_;
  Ogre::SceneNode * name_node_;

  float distance_to_parent_;
  Ogre::Quaternion arrow_orientation_;

  tf2::TimePoint last_update_;
  tf2::TimePoint last_time_to_fixed_;

  rviz_common::properties::VectorProperty * rel_position_property_;
  rviz_common::properties::QuaternionProperty * rel_orientation_property_;
  rviz_common::properties::VectorProperty * position_property_;
  rviz_common::properties::QuaternionProperty * orientation_property_;
  rviz_common::properties::StringProperty * parent_property_;
  rviz_common::properties::BoolProperty * enabled_property_;

  rviz_common::properties::Property * tree_property_;
};

}  // namespace displays
}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__TF__FRAME_INFO_HPP_
