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

#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__TF__TF_DISPLAY_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__TF__TF_DISPLAY_HPP_

#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include <OgreQuaternion.h>
#include <OgreVector.h>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2/buffer_core.h"
#include "tf2/time.h"

#include "rviz_common/interaction/forwards.hpp"
#include "rviz_common/display.hpp"

#include "rviz_default_plugins/transformation/transformer_guard.hpp"
#include "rviz_default_plugins/transformation/tf_frame_transformer.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

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
}  // namespace properties
}  // namespace rviz_common

namespace rviz_default_plugins
{
namespace displays
{

class FrameInfo;

class FrameSelectionHandler;

typedef std::shared_ptr<FrameSelectionHandler> FrameSelectionHandlerPtr;
typedef std::set<FrameInfo *> S_FrameInfo;

/** @brief Displays a visual representation of the TF hierarchy. */
class RVIZ_DEFAULT_PLUGINS_PUBLIC TFDisplay : public rviz_common::Display
{
  Q_OBJECT

public:
  TFDisplay();

  ~TFDisplay() override;

  void update(float wall_dt, float ros_dt) override;

protected:
  void onInitialize() override;
  void load(const rviz_common::Config & config) override;
  void fixedFrameChanged() override;
  void reset() override;

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

  void onEnable() override;
  void onDisable() override;

  Ogre::SceneNode * root_node_;
  Ogre::SceneNode * names_node_;
  Ogre::SceneNode * arrows_node_;
  Ogre::SceneNode * axes_node_;

  typedef std::map<std::string, FrameInfo *> M_FrameInfo;
  M_FrameInfo frames_;

  typedef std::map<std::string, bool> M_EnabledState;
  M_EnabledState frame_config_enabled_state_;

  float update_timer_;

  rviz_common::properties::BoolProperty * show_names_property_;
  rviz_common::properties::BoolProperty * show_arrows_property_;
  rviz_common::properties::BoolProperty * show_axes_property_;
  rviz_common::properties::FloatProperty * update_rate_property_;
  rviz_common::properties::FloatProperty * frame_timeout_property_;
  rviz_common::properties::BoolProperty * all_enabled_property_;

  rviz_common::properties::FloatProperty * scale_property_;

  rviz_common::properties::Property * frames_category_;
  rviz_common::properties::Property * tree_category_;

  bool changing_single_frame_enabled_state_;

  std::unique_ptr<rviz_default_plugins::transformation::TransformerGuard<
      rviz_default_plugins::transformation::TFFrameTransformer>> transformer_guard_;

  void updateRelativePositionAndOrientation(
    const FrameInfo * frame, std::shared_ptr<tf2::BufferCore> tf_buffer) const;

  void logTransformationException(
    const std::string & parent_frame,
    const std::string & child_frame,
    const std::string & message = "") const;

  void updateParentArrowIfTransformExists(FrameInfo * frame, const Ogre::Vector3 & position) const;

  bool hasNoTreePropertyOrParentChanged(
    const FrameInfo * frame, const std::string & old_parent) const;
  void updateParentTreeProperty(FrameInfo * frame) const;

  void deleteObsoleteFrames(std::set<FrameInfo *> & current_frames);
  S_FrameInfo createOrUpdateFrames(const std::vector<std::string> & frames);

  friend class FrameInfo;
};

}  // namespace displays
}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__TF__TF_DISPLAY_HPP_
