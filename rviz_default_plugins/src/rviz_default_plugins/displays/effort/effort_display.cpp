/*
 * Copyright (c) 2023, Open Source Robotics Foundation, Inc.
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

#include "rviz_default_plugins/displays/effort/effort_display.hpp"

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <urdf/model.h>

#include <QString>

#include <chrono>
#include <cstddef>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <rviz_common/validate_floats.hpp>

#include <rviz_common/properties/property.hpp>
#include <rviz_rendering/objects/effort_visual.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rclcpp/logging.hpp>

using namespace std::chrono_literals;

namespace rviz_default_plugins
{
namespace displays
{
JointInfo::JointInfo(const std::string & name, rviz_common::properties::Property * parent_category)
{
  name_ = name;
  effort_ = 0;
  max_effort_ = 0;

  category_ = new rviz_common::properties::Property(
    QString::fromStdString(name_), true, "", parent_category,
    SLOT(updateVisibility()), this);

  effort_property_ = new rviz_common::properties::FloatProperty(
    "Effort", 0, "Effort value of this joint.", category_);
  effort_property_->setReadOnly(true);

  max_effort_property_ =
    new rviz_common::properties::FloatProperty(
    "Max Effort", 0, "Max Effort value of this joint.", category_);
  max_effort_property_->setReadOnly(true);
}

JointInfo::~JointInfo()
{
}

std::shared_ptr<JointInfo> EffortDisplay::getJointInfo(const std::string & joint)
{
  M_JointInfo::iterator it = joints_.find(joint);
  if (it == joints_.end()) {
    return nullptr;
  }

  return it->second;
}

void JointInfo::updateVisibility()
{
}

void JointInfo::setEffort(double e)
{
  effort_property_->setFloat(e);
  effort_ = e;
}

void JointInfo::setMaxEffort(double m)
{
  max_effort_property_->setFloat(m);
  max_effort_ = m;
}

bool JointInfo::getEnabled() const
{
  return category_->getValue().toBool();
}

EffortDisplay::EffortDisplay()
: rviz_common::MessageFilterDisplay<sensor_msgs::msg::JointState>()
{
  alpha_property_ = new rviz_common::properties::FloatProperty(
    "Alpha", 1.0f, "0 is fully transparent, 1.0 is fully opaque.",
    this, SLOT(updateColorAndAlpha()));

  width_property_ = new rviz_common::properties::FloatProperty(
    "Width", 0.02f, "Width to drow effort circle", this,
    SLOT(updateColorAndAlpha()));

  scale_property_ = new rviz_common::properties::FloatProperty(
    "Scale", 1.0f, "Scale to draw effort circle", this,
    SLOT(updateColorAndAlpha()));

  history_length_property_ = new rviz_common::properties::IntProperty(
    "History Length", 1, "Number of prior measurements to display.", this,
    SLOT(updateHistoryLength()));

  history_length_property_->setMin(1);
  history_length_property_->setMax(100000);

  robot_description_property_ =
    new rviz_common::properties::StringProperty(
    "Robot Description", "/robot_description",
    "Name of the topic from which to load the robot "
    "description.",
    this, SLOT(updateRobotDescription()));

  tf_prefix_property_ = new rviz_common::properties::StringProperty(
    "TF Prefix", "",
    "Robot Model normally assumes the link name is the same as the tf frame name. "
    "This option allows you to set a prefix.  Mainly useful for multi-robot situations.",
    this, SLOT(updateTfPrefix()));

  joints_category_ = new rviz_common::properties::Property("Joints", QVariant(), "", this);
}

EffortDisplay::~EffortDisplay()
{
}

void EffortDisplay::onInitialize()
{
  MFDClass::onInitialize();
  updateHistoryLength();
}

// Set the number of past visuals to show.
void EffortDisplay::updateHistoryLength()
{
  while (visuals_.size() > static_cast<size_t>(history_length_property_->getInt())) {
    visuals_.pop_front();
  }
}

// Clear the visuals by deleting their objects.
void EffortDisplay::reset()
{
  MFDClass::reset();
  visuals_.clear();
}

void EffortDisplay::load(const rviz_common::Config & config)
{
  rviz_common::Display::load(config);
}

void EffortDisplay::topic_callback(const std_msgs::msg::String & msg)
{
  robot_description_ = msg.data;
  robot_model_ = std::shared_ptr<urdf::Model>(new urdf::Model());
  if (!robot_model_->initString(robot_description_)) {
    setStatus(
      rviz_common::properties::StatusProperty::Error,
      "URDF", "Unable to parse robot model description!");
    return;
  }
  setStatus(rviz_common::properties::StatusProperty::Ok, "URDF", "Robot model parsed Ok");

  for (std::map<std::string, urdf::JointSharedPtr>::iterator it = robot_model_->joints_.begin();
    it != robot_model_->joints_.end(); it++)
  {
    urdf::JointSharedPtr joint = it->second;
    if (joint->type == urdf::Joint::REVOLUTE || joint->type == 2) {
      std::string joint_name = it->first;
      urdf::JointLimitsSharedPtr limit = joint->limits;
      if (limit) {
        joints_[joint_name] = std::make_shared<JointInfo>(joint_name, joints_category_);
        joints_[joint_name]->setMaxEffort(limit->effort);
      } else {
        RCLCPP_WARN(
          context_->getRosNodeAbstraction().lock()->get_raw_node()->get_logger(),
          "Joint '%s' has no <limit> tag in URDF. Effort plugin needs to know the effort "
          "limit to determine the size of the corresponding visual marker. "
          "Effort display for this joint will be inhibited.", joint_name.c_str());
      }
    }
  }
}

void EffortDisplay::subscribeToRobotDescription()
{
  if (this->robot_description_topic_ == robot_description_property_->getStdString()) {
    return;
  }
  this->robot_description_topic_ = robot_description_property_->getStdString();

  using std::placeholders::_1;

  try {
    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.event_callbacks.message_lost_callback =
      [&](rclcpp::QOSMessageLostInfo & info)
      {
        std::ostringstream sstm;
        sstm << "Some messages were lost:\n>\tNumber of new lost messages: " <<
          info.total_count_change << " \n>\tTotal number of messages lost: " <<
          info.total_count;
        setStatus(
          rviz_common::properties::StatusLevel::Warn,
          "Topic",
          QString(sstm.str().c_str()));
      };

    this->subscription_ = context_->getRosNodeAbstraction().lock()->get_raw_node()->
      template create_subscription<std_msgs::msg::String>(
      robot_description_property_->getStdString(),
      rclcpp::QoS(1).transient_local(),
      std::bind(&EffortDisplay::topic_callback, this, _1),
      sub_opts);
    setStatus(rviz_common::properties::StatusLevel::Ok, "Array Topic", "OK");
  } catch (rclcpp::exceptions::InvalidTopicNameError & e) {
    setStatus(
      rviz_common::properties::StatusLevel::Error,
      "Topic",
      QString("Error subscribing: ") + e.what());
  }
}

void EffortDisplay::load()
{
  this->subscribeToRobotDescription();
}

void EffortDisplay::update(float wall_dt, float ros_dt)
{
  (void) wall_dt;
  (void) ros_dt;
}

// Set the current color and alpha values for each visual.
void EffortDisplay::updateColorAndAlpha()
{
  float width = width_property_->getFloat();
  float scale = scale_property_->getFloat();

  for (size_t i = 0; i < visuals_.size(); i++) {
    visuals_[i]->setWidth(width);
    visuals_[i]->setScale(scale);
  }
}

std::string concat(const std::string & prefix, const std::string & frame)
{
  if (prefix.empty()) {
    return frame;
  }

  std::string composite = prefix;
  composite.append("/");
  composite.append(frame);
  return composite;
}

void EffortDisplay::processMessage(sensor_msgs::msg::JointState::ConstSharedPtr msg)
{
  // Robot model might not be loaded yet
  if (!robot_model_) {
    setStatus(
      rviz_common::properties::StatusLevel::Error,
      "Process message",
      QString("Robot model might not be loaded yet"));
    return;
  }
  // We are keeping a circular buffer of visual pointers.  This gets
  // the next one, or creates and stores it if the buffer is not full
  std::shared_ptr<rviz_rendering::EffortVisual> visual;
  if (visuals_.size() == static_cast<size_t>(history_length_property_->getInt())) {
    visual = visuals_.front();
    visual->setWidth(width_property_->getFloat());
    visual->setScale(scale_property_->getFloat());
  } else {
    visual = std::make_shared<rviz_rendering::EffortVisual>(
      context_->getSceneManager(), scene_node_,
      width_property_->getFloat(), scale_property_->getFloat());
  }

  if (visuals_.size() >= static_cast<size_t>(history_length_property_->getInt())) {
    visuals_.pop_front();
  }

  std::vector<std::string> joints;
  size_t joint_num = msg->name.size();
  if (joint_num != msg->effort.size()) {
    setStatus(
      rviz_common::properties::StatusProperty::Error, "Topic",
      "Received a joint state msg with different joint names and efforts size!");
    return;
  }
  for (size_t i = 0; i < joint_num; ++i) {
    const std::string & joint_name = msg->name[i];
    std::shared_ptr<JointInfo> joint_info = getJointInfo(joint_name);
    if (!joint_info) {
      continue;  // skip joints..
    }

    rclcpp::Time msg_time(msg->header.stamp, RCL_ROS_TIME);

    // update effort property
    joint_info->setEffort(msg->effort[i]);
    joint_info->last_update_ = msg_time;

    const urdf::Joint * joint = robot_model_->getJoint(joint_name).get();
    int joint_type = joint->type;
    if (joint_type == urdf::Joint::REVOLUTE) {
      std::string tf_frame_id = concat(
        tf_prefix_property_->getStdString(), joint->child_link_name);
      Ogre::Quaternion orientation;
      Ogre::Vector3 position;

      // Call rviz::FrameManager to get the transform from the fixed frame to the joint's frame.
      if (!context_->getFrameManager()->getTransform(
          tf_frame_id, msg_time, position, orientation))
      {
        setStatus(
          rviz_common::properties::StatusProperty::Error,
          QString::fromStdString(joint_name),
          QString("Error transforming from frame '%1' to frame '%2'")
          .arg(tf_frame_id.c_str(), qPrintable(fixed_frame_)));
        continue;
      }
      tf2::Vector3 axis_joint(joint->axis.x, joint->axis.y, joint->axis.z);
      tf2::Vector3 axis_z(0, 0, 1);
      tf2::Quaternion axis_rotation(axis_joint.cross(axis_z), axis_joint.angle(axis_z));
      if (std::isnan(axis_rotation.x()) ||
        std::isnan(axis_rotation.y()) ||
        std::isnan(axis_rotation.z()))
      {
        axis_rotation = tf2::Quaternion::getIdentity();
      }

      tf2::Quaternion axis_orientation(orientation.x, orientation.y, orientation.z, orientation.w);
      tf2::Quaternion axis_rot = axis_orientation * axis_rotation;
      Ogre::Quaternion joint_orientation(Ogre::Real(axis_rot.w()), Ogre::Real(axis_rot.x()),
        Ogre::Real(axis_rot.y()), Ogre::Real(axis_rot.z()));
      visual->setFramePosition(joint_name, position);
      visual->setFrameOrientation(joint_name, joint_orientation);
      visual->setFrameEnabled(joint_name, joint_info->getEnabled());

      if (!rviz_common::validateFloats(joint_info->getEffort())) {
        setStatus(
          rviz_common::properties::StatusProperty::Error,
          QString::fromStdString(joint_name),
          QString("Invalid effort: %1").arg(joint_info->getEffort()));
        visual->setFrameEnabled(joint_name, false);
      } else {
        setStatus(
          rviz_common::properties::StatusProperty::Ok,
          QString::fromStdString(joint_name),
          QString());
      }
      visual->setEffort(joint_name, joint_info->getEffort(), joint_info->getMaxEffort());
    }
  }
  visuals_.push_back(visual);
}

void EffortDisplay::updateRobotDescription()
{
  if (isEnabled()) {
    load();
    context_->queueRender();
  }
}

void EffortDisplay::onEnable()
{
  load();
}

void EffortDisplay::onDisable()
{
  clear();
}

void EffortDisplay::clear()
{
  clearStatuses();
  robot_description_.clear();
}

void EffortDisplay::updateTfPrefix()
{
  clearStatuses();
  context_->queueRender();
}

}  // namespace displays
}  // namespace rviz_default_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::EffortDisplay, rviz_common::Display)
