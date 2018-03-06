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

#include "robot_model_display.hpp"

#include <memory>
#include <string>

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include <QFile>

// TODO(Martin-Idel-SI): Upgrade to tinyxml2 once supported by urdf
#include <tinyxml.h>
#include "urdf/model.h"

#include "tf2_ros/transform_listener.h"

#include "../../robot/robot.hpp"
#include "../../robot/tf_link_updater.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/file_picker_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/property.hpp"
#include "rviz_common/properties/string_property.hpp"

namespace rviz_default_plugins
{
namespace displays
{

using rviz_common::properties::EnumProperty;
using rviz_common::properties::FilePickerProperty;
using rviz_common::properties::FloatProperty;
using rviz_common::properties::StatusProperty;
using rviz_common::properties::StringProperty;
using rviz_common::properties::Property;

void linkUpdaterStatusFunction(
  StatusProperty::Level level,
  const std::string & link_name,
  const std::string & text,
  RobotModelDisplay * display)
{
  display->setStatus(level, QString::fromStdString(link_name), QString::fromStdString(text));
}

enum DescriptionSource
{
  TOPIC, FILE
};

RobotModelDisplay::RobotModelDisplay()
: has_new_transforms_(false),
  time_since_last_transform_(0.0f)
{
  visual_enabled_property_ = new Property("Visual Enabled", true,
      "Whether to display the visual representation of the robot.",
      this, SLOT(updateVisualVisible()));

  collision_enabled_property_ = new Property("Collision Enabled", false,
      "Whether to display the collision representation of the robot.",
      this, SLOT(updateCollisionVisible()));

  update_rate_property_ = new FloatProperty("Update Interval", 0,
      "Interval at which to update the links, in seconds. "
      " 0 means to update every update cycle.",
      this);
  update_rate_property_->setMin(0);

  alpha_property_ = new FloatProperty("Alpha", 1,
      "Amount of transparency to apply to the links.",
      this, SLOT(updateAlpha()));
  alpha_property_->setMin(0.0);
  alpha_property_->setMax(1.0);

  description_source_property_ = new EnumProperty("Description Source", "Topic",
      "Source to get the robot description from.", this, SLOT(updatePropertyVisibility()));
  description_source_property_->addOption("Topic", DescriptionSource::TOPIC);
  description_source_property_->addOption("File", DescriptionSource::FILE);

  description_file_property_ = new FilePickerProperty("Description File", "",
      "Path to the robot description.",
      this, SLOT(updateRobotDescription()));

  this->moveChild(topic_property_->rowNumberInParent(), this->numChildren() - 1);
  topic_property_->setDescription("Topic where filepath to urdf is published.");
  topic_property_->setName("Description Topic");

  unreliable_property_->setHidden(true);

  tf_prefix_property_ = new StringProperty("TF Prefix", "",
      "Robot Model normally assumes the link name is the same as the tf frame name. "
      " This option allows you to set a prefix.  Mainly useful for multi-robot situations.",
      this, SLOT(updateTfPrefix()));
}

RobotModelDisplay::~RobotModelDisplay() = default;

void RobotModelDisplay::onInitialize()
{
  RTDClass::onInitialize();
  robot_ = std::make_unique<robot::Robot>(
    scene_node_, context_, "Robot: " + getName().toStdString(), this);

  updateVisualVisible();
  updateCollisionVisible();
  updateAlpha();
  updatePropertyVisibility();
}

void RobotModelDisplay::updateAlpha()
{
  robot_->setAlpha(alpha_property_->getFloat());
  context_->queueRender();
}

void RobotModelDisplay::updatePropertyVisibility()
{
  if (description_source_property_->getOptionInt() == DescriptionSource::TOPIC) {
    description_file_property_->setHidden(true);
    topic_property_->setHidden(false);
    clear();
    updateTopic();
  } else if (description_source_property_->getOptionInt() == DescriptionSource::FILE) {
    topic_property_->setHidden(true);
    description_file_property_->setHidden(false);
    RTDClass::unsubscribe();
    updateRobotDescription();
  }
}

void RobotModelDisplay::updateRobotDescription()
{
  if (isEnabled()) {
    load_urdf();
    context_->queueRender();
  }
}

void RobotModelDisplay::updateTopic()
{
  if (isEnabled()) {
    RTDClass::updateTopic();
  }
}

void RobotModelDisplay::updateVisualVisible()
{
  robot_->setVisualVisible(visual_enabled_property_->getValue().toBool());
  context_->queueRender();
}

void RobotModelDisplay::updateCollisionVisible()
{
  robot_->setCollisionVisible(collision_enabled_property_->getValue().toBool());
  context_->queueRender();
}

void RobotModelDisplay::updateTfPrefix()
{
  clearStatuses();
  context_->queueRender();
}

void RobotModelDisplay::load_urdf()
{
  if (description_source_property_->getOptionInt() == DescriptionSource::FILE &&
    !description_file_property_->getString().isEmpty())
  {
    load_urdf_from(description_file_property_->getStdString());
  } else {
    clear();
  }
}

void RobotModelDisplay::load_urdf_from(const std::string & filepath)
{
  std::string content;
  QFile urdf_file(QString::fromStdString(filepath));
  if (urdf_file.open(QIODevice::ReadOnly)) {
    content = urdf_file.readAll().toStdString();
    urdf_file.close();
  }
  if (content.empty()) {
    clear();
    setStatus(StatusProperty::Error, "URDF", "URDF is empty");
    return;
  }

  if (content == robot_description_) {
    return;
  }

  robot_description_ = content;

  TiXmlDocument doc;
  doc.Parse(robot_description_.c_str());
  if (!doc.RootElement() ) {
    clear();
    setStatus(StatusProperty::Error, "URDF", "URDF failed XML parse");
    return;
  }

  urdf::Model descr;
  if (!descr.initXml(doc.RootElement())) {
    clear();
    setStatus(StatusProperty::Error, "URDF", "URDF failed Model parse");
    return;
  }

  setStatus(StatusProperty::Ok, "URDF", "URDF parsed OK");
  robot_->load(descr);
  updateRobot();
}

void RobotModelDisplay::updateRobot()
{
  robot_->update(robot::TFLinkUpdater(
      context_->getFrameManager(),
      [this](auto arg1, auto arg2, auto arg3) {linkUpdaterStatusFunction(arg1, arg2, arg3, this);},
      tf_prefix_property_->getStdString()));
}

void RobotModelDisplay::onEnable()
{
  if (description_source_property_->getOptionInt() == DescriptionSource::TOPIC) {
    RTDClass::onEnable();
  }
  load_urdf();
  robot_->setVisible(true);
}

void RobotModelDisplay::onDisable()
{
  RTDClass::onDisable();
  robot_->setVisible(false);
  clear();
}

void RobotModelDisplay::update(float wall_dt, float ros_dt)
{
  (void) ros_dt;
  time_since_last_transform_ += wall_dt;
  float rate = update_rate_property_->getFloat();
  bool update = rate < 0.0001f || time_since_last_transform_ >= rate * 1000000000;

  if (has_new_transforms_ || update) {
    updateRobot();
    context_->queueRender();

    has_new_transforms_ = false;
    time_since_last_transform_ = 0.0f;
  }
}

void RobotModelDisplay::fixedFrameChanged()
{
  has_new_transforms_ = true;
}

void RobotModelDisplay::clear()
{
  robot_->clear();
  clearStatuses();
  robot_description_.clear();
}

void RobotModelDisplay::reset()
{
  RTDClass::reset();
  has_new_transforms_ = true;
}

void RobotModelDisplay::processMessage(std_msgs::msg::String::ConstSharedPtr msg)
{
  std::string filepath = msg->data;
  load_urdf_from(filepath);
}

}  // namespace displays
}  // namespace rviz_default_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::RobotModelDisplay, rviz_common::Display)
