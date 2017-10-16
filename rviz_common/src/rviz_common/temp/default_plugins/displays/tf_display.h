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

#ifndef RVIZ_COMMON__TEMP__DEFAULT_PLUGINS__DISPLAYS__TF_DISPLAY_H_
#define RVIZ_COMMON__TEMP__DEFAULT_PLUGINS__DISPLAYS__TF_DISPLAY_H_

#include <map>
#include <memory>
#include <set>
#include <string>

#include <OgreQuaternion.h>
#include <OgreVector3.h>

#include <tf2/time.h> // NOLINT: cpplint is unable to handle the include order here

#include "../../../selection/forwards.hpp"
#include "../../../display.hpp"

namespace Ogre
{
class SceneNode;
}

namespace rviz_rendering
{
class Arrow;
class Axes;
class MovableText;
}

namespace rviz_common
{

namespace properties
{
class BoolProperty;
class FloatProperty;
class QuaternionProperty;
class StringProperty;
class VectorProperty;
}

class FrameInfo;
class FrameSelectionHandler;
typedef std::shared_ptr<FrameSelectionHandler> FrameSelectionHandlerPtr;

/** @brief Displays a visual representation of the TF hierarchy. */
class TFDisplay : public Display
{
  Q_OBJECT

public:
  TFDisplay();
  virtual ~TFDisplay();

  virtual void update(float wall_dt, float ros_dt);

protected:
  // Overrides from Display
  virtual void onInitialize();
  virtual void load(const Config & config);
  virtual void fixedFrameChanged();
  virtual void reset();

private Q_SLOTS:
  void updateShowAxes();
  void updateShowArrows();
  void updateShowNames();
  void allEnabledChanged();

private:
  void updateFrames();
  FrameInfo * createFrame(const std::string & frame);
  void updateFrame(FrameInfo * frame);
  void deleteFrame(FrameInfo * frame, bool delete_properties);

  FrameInfo * getFrameInfo(const std::string & frame);

  void clear();

  // overrides from Display
  virtual void onEnable();
  virtual void onDisable();

  Ogre::SceneNode * root_node_;
  Ogre::SceneNode * names_node_;
  Ogre::SceneNode * arrows_node_;
  Ogre::SceneNode * axes_node_;

  typedef std::map<std::string, FrameInfo *> M_FrameInfo;
  M_FrameInfo frames_;

  typedef std::map<std::string, bool> M_EnabledState;
  M_EnabledState frame_config_enabled_state_;

  float update_timer_;

  BoolProperty * show_names_property_;
  BoolProperty * show_arrows_property_;
  BoolProperty * show_axes_property_;
  properties::FloatProperty * update_rate_property_;
  properties::FloatProperty * frame_timeout_property_;
  BoolProperty * all_enabled_property_;

  properties::FloatProperty * scale_property_;

  Property * frames_category_;
  Property * tree_category_;

  bool changing_single_frame_enabled_state_;
  friend class FrameInfo;
};

/** @brief Internal class needed only by TFDisplay. */
class FrameInfo : public QObject
{
  Q_OBJECT

public:
  explicit FrameInfo(TFDisplay * display);

  /** @brief Set this frame to be visible or invisible. */
  void setEnabled(bool enabled);

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
  selection::CollObjectHandle axes_coll_;
  FrameSelectionHandlerPtr selection_handler_;
  rviz_rendering::Arrow * parent_arrow_;
  rviz_rendering::MovableText * name_text_;
  Ogre::SceneNode * name_node_;

  float distance_to_parent_;
  Ogre::Quaternion arrow_orientation_;

  tf2::TimePoint last_update_;
  tf2::TimePoint last_time_to_fixed_;

  properties::VectorProperty * rel_position_property_;
  properties::QuaternionProperty * rel_orientation_property_;
  properties::VectorProperty * position_property_;
  properties::QuaternionProperty * orientation_property_;
  properties::StringProperty * parent_property_;
  properties::BoolProperty * enabled_property_;

  properties::Property * tree_property_;
};

}  // end namespace rviz_common

#endif  // RVIZ_COMMON__TEMP__DEFAULT_PLUGINS__DISPLAYS__TF_DISPLAY_H_
