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

#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__ROBOT_MODEL__ROBOT_MODEL_DISPLAY_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__ROBOT_MODEL__ROBOT_MODEL_DISPLAY_HPP_

#include <map>
#include <memory>
#include <string>

#include <OgreVector.h>

#include "std_msgs/msg/string.hpp"

#include "rviz_common/ros_topic_display.hpp"

#include "rviz_default_plugins/transformation/transformer_guard.hpp"
#include "rviz_default_plugins/transformation/tf_frame_transformer.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

namespace Ogre
{
class Entity;
class SceneNode;
}

namespace rviz_rendering
{
class Axes;
}

namespace rviz_common
{
namespace properties
{
class EnumProperty;
class FilePickerProperty;
class FloatProperty;
class Property;
class StringProperty;
}  // namespace properties
}  // namespace rviz_common

namespace rviz_default_plugins
{

namespace robot
{
class Robot;
}

namespace displays
{

/**
 * \class RobotModelDisplay
 * \brief Uses a robot xml description to display the pieces of a robot at the transforms
 * broadcast by rosTF
 */
class RVIZ_DEFAULT_PLUGINS_PUBLIC RobotModelDisplay : public
  rviz_common::RosTopicDisplay<std_msgs::msg::String>
{
  Q_OBJECT

public:
  RobotModelDisplay();
  ~RobotModelDisplay() override;

  // Overrides from Display
  void onInitialize() override;
  void update(float wall_dt, float ros_dt) override;
  void fixedFrameChanged() override;
  void reset() override;

  void clear();

private Q_SLOTS:
  void updateVisualVisible();
  void updateCollisionVisible();
  void updateTfPrefix();
  void updateAlpha();
  void updatePropertyVisibility();
  void updateRobotDescription();
  void updateMassVisible();
  void updateInertiaVisible();

  void updateTopic() override;

protected:
  /** @brief Loads a URDF from the ros-param named by our
   * "Robot Description" property, iterates through the links, and
   * loads any necessary models. */
  virtual void load_urdf();
  virtual void load_urdf_from_file(const std::string & filepath);
  virtual void load_urdf_from_string(const std::string & robot_description);
  void display_urdf_content();
  void updateRobot();

  void processMessage(std_msgs::msg::String::ConstSharedPtr msg) override;

  // overrides from Display
  void onEnable() override;
  void onDisable() override;

  std::unique_ptr<robot::Robot> robot_;                 ///< Handles actually drawing the robot

  bool has_new_transforms_;      ///< Callback sets this to tell our update function
  ///< it needs to update the transforms

  float time_since_last_transform_;

  std::string robot_description_;

  rviz_common::properties::Property * visual_enabled_property_;
  rviz_common::properties::Property * collision_enabled_property_;
  rviz_common::properties::FloatProperty * update_rate_property_;
  rviz_common::properties::EnumProperty * description_source_property_;
  rviz_common::properties::FilePickerProperty * description_file_property_;
  rviz_common::properties::FloatProperty * alpha_property_;
  rviz_common::properties::StringProperty * tf_prefix_property_;

  rviz_common::properties::Property * mass_properties_;
  rviz_common::properties::Property * mass_enabled_property_;
  rviz_common::properties::Property * inertia_enabled_property_;

  std::unique_ptr<rviz_default_plugins::transformation::TransformerGuard<
      rviz_default_plugins::transformation::TFFrameTransformer>> transformer_guard_;
};

}  // namespace displays
}  // namespace rviz_default_plugins
#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__ROBOT_MODEL__ROBOT_MODEL_DISPLAY_HPP_
