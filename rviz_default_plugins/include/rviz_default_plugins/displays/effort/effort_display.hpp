// Copyright (c) 2023, Open Source Robotics Foundation, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__EFFORT__EFFORT_DISPLAY_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__EFFORT__EFFORT_DISPLAY_HPP_


#include <deque>
#include <map>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/ros_topic_display.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_rendering/objects/effort_visual.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <urdf/model.hpp>

#include "rviz_default_plugins/visibility_control.hpp"

namespace rviz_default_plugins
{
namespace displays
{
class JointInfo : public QObject
{
  Q_OBJECT

public:
  JointInfo(const std::string & name, rviz_common::properties::Property * parent_category);
  ~JointInfo() override;

  void setEffort(double e);
  inline double getEffort()
  {
    return effort_;
  }
  void setMaxEffort(double m);
  inline double getMaxEffort()
  {
    return max_effort_;
  }
  bool getEnabled() const;

  rclcpp::Time last_update_;

public Q_SLOTS:
  void updateVisibility();

private:
  std::string name_;
  double effort_, max_effort_;

  rviz_common::properties::Property * category_;
  rviz_common::properties::FloatProperty * effort_property_;
  rviz_common::properties::FloatProperty * max_effort_property_;
};

class RVIZ_DEFAULT_PLUGINS_PUBLIC EffortDisplay
  : public rviz_common::RosTopicDisplay<sensor_msgs::msg::JointState>
{
  Q_OBJECT

public:
  EffortDisplay();
  ~EffortDisplay() override;

  // Overrides of public virtual functions from the Display class.
  void onInitialize() override;
  void reset() override;

  void load(const rviz_common::Config & config) override;

  void update(float wall_dt, float ros_dt) override;

private Q_SLOTS:
  // Helper function to apply color and alpha to all visuals.
  void updateColorAndAlpha();
  void updateHistoryLength();
  void updateRobotDescription();
  void updateTfPrefix();

private:
  std::shared_ptr<JointInfo> getJointInfo(const std::string & joint);
  void subscribeToRobotDescription();

protected:
  // overrides from Display
  void onEnable() override;
  void onDisable() override;
  void processMessage(sensor_msgs::msg::JointState::ConstSharedPtr msg) override;
  void processTypeErasedMessage(std::shared_ptr<const void> type_erased_msg) override;

  void load();
  void clear();

  // The object for urdf model
  std::shared_ptr<urdf::Model> robot_model_;

  std::string robot_description_;
  std::string robot_description_topic_;

private:
  void topic_callback(const std_msgs::msg::String & msg);

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;

  // Storage for the list of visuals.  It is a circular buffer where
  // data gets popped from the front (oldest) and pushed to the back (newest)
  std::deque<std::shared_ptr<rviz_rendering::EffortVisual>> visuals_;

  typedef std::map<std::string, std::shared_ptr<JointInfo>> M_JointInfo;
  M_JointInfo joints_;

  // Property objects for user-editable properties.
  rviz_common::properties::FloatProperty * alpha_property_, * width_property_, * scale_property_;
  rviz_common::properties::IntProperty * history_length_property_;

  rviz_common::properties::StringProperty * robot_description_property_;
  rviz_common::properties::StringProperty * tf_prefix_property_;
  rviz_common::properties::Property * joints_category_;
  rviz_common::properties::BoolProperty * all_enabled_property_;
};

}  // namespace displays
}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__EFFORT__EFFORT_DISPLAY_HPP_
