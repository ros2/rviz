/*
 * Copyright (c) 2013, Willow Garage, Inc.
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

#include "rviz_default_plugins/robot/robot_joint.hpp"

#include <memory>
#include <string>
#include <vector>

#include "rviz_default_plugins/robot/robot.hpp"

#include "rviz_common/load_resource.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/quaternion_property.hpp"
#include "rviz_common/properties/string_property.hpp"
#include "rviz_common/properties/vector_property.hpp"
#include "rviz_rendering/objects/arrow.hpp"
#include "rviz_rendering/objects/axes.hpp"

using rviz_rendering::Axes;
using rviz_rendering::Arrow;

namespace rviz_default_plugins
{
namespace robot
{

using rviz_common::properties::Property;
using rviz_common::properties::VectorProperty;
using rviz_common::properties::QuaternionProperty;
using rviz_common::properties::StringProperty;
using rviz_common::properties::FloatProperty;

RobotJoint::RobotJoint(Robot * robot, const urdf::JointConstSharedPtr & joint)
: RobotElementBaseClass(robot, joint->name),
  parent_link_name_(joint->parent_link_name),
  child_link_name_(joint->child_link_name),
  has_decendent_links_with_geometry_(true),
  doing_set_checkbox_(false),
  axis_(nullptr)
{
  robot_element_property_ = new Property(
    name_.c_str(),
    true,
    "",
    nullptr,
    SLOT(updateChildVisibility()),
    this);
  robot_element_property_->setIcon(
    rviz_common::loadPixmap(
      "package://rviz_default_plugins/icons/classes/RobotJoint.png"));

  details_ = new Property("Details", QVariant(), "", nullptr);

  axes_property_ = new Property(
    "Show Axes",
    false,
    "Enable/disable showing the axes of this joint.",
    robot_element_property_,
    SLOT(updateAxes()),
    this);

  position_property_ = new VectorProperty(
    "Position",
    Ogre::Vector3::ZERO,
    "Position of this joint, in the current Fixed Frame.  (Not editable)",
    robot_element_property_);
  position_property_->setReadOnly(true);

  orientation_property_ = new QuaternionProperty(
    "Orientation",
    Ogre::Quaternion::IDENTITY,
    "Orientation of this joint, in the current Fixed Frame.  (Not editable)",
    robot_element_property_);
  orientation_property_->setReadOnly(true);

  std::string type = getType(joint);

  type_property_ = new StringProperty(
    "Type",
    QString::fromStdString(type),
    "Type of this joint.  (Not editable)",
    robot_element_property_);
  type_property_->setReadOnly(true);

  showLimitProperties(joint);
  showAxisForMovingJoints(joint, type);

  robot_element_property_->collapse();

  const urdf::Vector3 & pos = joint->parent_to_joint_origin_transform.position;
  const urdf::Rotation & rot = joint->parent_to_joint_origin_transform.rotation;
  joint_origin_pos_ = Ogre::Vector3(pos.x, pos.y, pos.z);
  joint_origin_rot_ = Ogre::Quaternion(rot.w, rot.x, rot.y, rot.z);
}


RobotJoint::~RobotJoint()
{
  delete details_;
  delete robot_element_property_;
}

void RobotJoint::setTransforms(
  const Ogre::Vector3 & parent_link_position,
  const Ogre::Quaternion & parent_link_orientation)
{
  Ogre::Vector3 position = parent_link_position + parent_link_orientation * joint_origin_pos_;
  Ogre::Quaternion orientation = parent_link_orientation * joint_origin_rot_;

  position_property_->setVector(position);
  orientation_property_->setQuaternion(orientation);

  if (axes_) {
    axes_->setPosition(position);
    axes_->setOrientation(orientation);
  }
  if (axis_) {
    axis_->setPosition(position);
    axis_->setOrientation(orientation);
    axis_->setDirection(parent_link_orientation * axis_property_->getVector());
  }
}

RobotJoint * RobotJoint::getParentJoint()
{
  RobotLink * parent_link = robot_->getLink(parent_link_name_);
  if (!parent_link) {
    return nullptr;
  }

  const std::string & parent_joint_name = parent_link->getParentJointName();
  if (parent_joint_name.empty()) {
    return nullptr;
  }

  return robot_->getJoint(parent_joint_name);
}

void RobotJoint::setJointPropertyDescription()
{
  int links_with_geom;
  int links_with_geom_checked;
  int links_with_geom_unchecked;
  getChildLinkState(links_with_geom, links_with_geom_checked, links_with_geom_unchecked, true);

  std::stringstream desc;
  desc <<
    "Joint <b>" << name_ <<
    "</b> with parent link <b>" << parent_link_name_ <<
    "</b> and child link <b>" << child_link_name_ <<
    "</b>.";

  if (links_with_geom == 0) {
    desc << "  This joint's descendents have NO geometry.";
    setJointCheckbox(QVariant());
    has_decendent_links_with_geometry_ = false;
  } else if (styleIsTree()) {
    desc << "  Check/uncheck to show/hide all links descended from this joint.";
    setJointCheckbox(links_with_geom_unchecked == 0);
    has_decendent_links_with_geometry_ = true;
  } else {
    getChildLinkState(links_with_geom, links_with_geom_checked, links_with_geom_unchecked, false);
    if (links_with_geom == 0) {
      desc << "  This joint's child link has NO geometry.";
      setJointCheckbox(QVariant());
      has_decendent_links_with_geometry_ = false;
    } else {
      desc << "  Check/uncheck to show/hide this joint's child link.";
      setJointCheckbox(links_with_geom_unchecked == 0);
      has_decendent_links_with_geometry_ = true;
    }
  }

  robot_element_property_->setDescription(desc.str().c_str());
}

void RobotJoint::setJointCheckbox(QVariant val)
{
  // setting doing_set_checkbox_ to true prevents updateChildVisibility() from
  // updating child link enables.
  doing_set_checkbox_ = true;
  robot_element_property_->setValue(val);
  doing_set_checkbox_ = false;
}

int RobotJoint::links_with_geom(
  RobotLink * link, int & links_with_geom_checked,
  int & links_with_geom_unchecked, int n_args, ...)
{
  va_list arg_ptr;
  va_start(arg_ptr, n_args);

  for (auto & child_joint_name : link->getChildJointNames()) {
    RobotJoint * child_joint = robot_->getJoint(child_joint_name);
    if (child_joint) {
      int child_links_with_geom;
      int child_links_with_geom_checked;
      int child_links_with_geom_unchecked;
      if (n_args == 1) {
        child_joint->getChildLinkState(
          child_links_with_geom, child_links_with_geom_checked,
          child_links_with_geom_unchecked, va_arg(arg_ptr, int));
      } else {
        child_joint->calculateJointCheckboxesRecursive(
          child_links_with_geom,
          child_links_with_geom_checked,
          child_links_with_geom_unchecked);
      }
      links_with_geom_checked += child_links_with_geom_checked;
      links_with_geom_unchecked += child_links_with_geom_unchecked;
    }
  }
  va_end(arg_ptr);
  return links_with_geom_checked + links_with_geom_unchecked;
}

void RobotJoint::calculateJointCheckboxesRecursive(
  int & links_with_geom,
  int & links_with_geom_checked,
  int & links_with_geom_unchecked)
{
  links_with_geom = 0;
  links_with_geom_checked = 0;
  links_with_geom_unchecked = 0;

  RobotLink * link =
    links_checked_and_unchecked(links_with_geom_checked, links_with_geom_unchecked);
  if (!link) {
    return;
  }

  links_with_geom = links_with_geom_checked + links_with_geom_unchecked;

  if (!styleIsTree()) {
    if (!links_with_geom) {
      setJointCheckbox(QVariant());
    } else {
      setJointCheckbox(links_with_geom_unchecked == 0);
    }
  }

  links_with_geom = this->links_with_geom(
    link, links_with_geom_checked, links_with_geom_unchecked, 0);

  if (styleIsTree()) {
    if (!links_with_geom) {
      setJointCheckbox(QVariant());
    } else {
      setJointCheckbox(links_with_geom_unchecked == 0);
    }
  }
}

RobotLink * RobotJoint::links_checked_and_unchecked(
  int & links_with_geom_checked, int & links_with_geom_unchecked)
{
  links_with_geom_checked = 0;
  links_with_geom_unchecked = 0;

  RobotLink * link = robot_->getLink(child_link_name_);
  if (!link) {
    return nullptr;
  }
  if (link->hasGeometry()) {
    bool checked = link->getLinkProperty()->getValue().toBool();
    links_with_geom_checked += checked ? 1 : 0;
    links_with_geom_unchecked += checked ? 0 : 1;
  }
  return link;
}

void RobotJoint::getChildLinkState(
  int & links_with_geom,
  int & links_with_geom_checked,
  int & links_with_geom_unchecked,
  bool recursive)
{
  links_with_geom = 0;
  RobotLink * link = this->links_checked_and_unchecked(
    links_with_geom_checked, links_with_geom_unchecked);
  if (!link) {
    return;
  }

  if (recursive) {
    this->links_with_geom(link, links_with_geom_checked, links_with_geom_unchecked, 1, recursive);
  }

  links_with_geom = links_with_geom_checked + links_with_geom_unchecked;
}

bool RobotJoint::getEnabled() const
{
  if (!hasDescendentLinksWithGeometry()) {
    return true;
  }
  return robot_element_property_->getValue().toBool();
}

bool RobotJoint::styleIsTree() const
{
  return details_->getParent() != nullptr;
}

void RobotJoint::updateChildVisibility()
{
  if (doing_set_checkbox_) {
    return;
  }

  if (!hasDescendentLinksWithGeometry()) {
    return;
  }

  bool visible = getEnabled();

  RobotLink * link = robot_->getLink(child_link_name_);
  if (link) {
    if (link->hasGeometry()) {
      link->getLinkProperty()->setValue(visible);
    }

    if (styleIsTree()) {
      for (auto & child_joint_name : link->getChildJointNames()) {
        RobotJoint * child_joint = robot_->getJoint(child_joint_name);
        if (child_joint) {
          child_joint->getJointProperty()->setValue(visible);
        }
      }
    }
  }
}

void RobotJoint::updateAxis()
{
  if (show_axis_property_->getValue().toBool()) {
    if (!axis_) {
      axis_ = std::make_unique<Arrow>(
        robot_->getSceneManager(), robot_->getOtherNode(), 0.15f, 0.05f, 0.05f, 0.08f);
      axis_->getSceneNode()->setVisible(getEnabled());

      axis_->setPosition(position_property_->getVector());
      axis_->setOrientation(orientation_property_->getQuaternion());

      // TODO(lucasw) store an Ogre::ColorValue and set it according to joint type.
      axis_->setColor(0.0f, 0.8f, 0.0f, 1.0f);
    }
  } else {
    axis_.reset(nullptr);
  }
}

std::string RobotJoint::getType(const urdf::JointConstSharedPtr & joint) const
{
  std::string type = "";
  if (joint->type == urdf::Joint::UNKNOWN) {
    type = "unknown";
  } else if (joint->type == urdf::Joint::REVOLUTE) {
    type = "revolute";
  } else if (joint->type == urdf::Joint::CONTINUOUS) {
    type = "continuous";
  } else if (joint->type == urdf::Joint::PRISMATIC) {
    type = "prismatic";
  } else if (joint->type == urdf::Joint::FLOATING) {
    type = "floating";
  } else if (joint->type == urdf::Joint::PLANAR) {
    type = "planar";
  } else if (joint->type == urdf::Joint::FIXED) {
    type = "fixed";
  }
  return type;
}

void RobotJoint::showLimitProperties(const urdf::JointConstSharedPtr & joint)
{
  if (joint->limits) {
    // continuous joints have lower limit and upper limits of zero,
    // which means this isn't very useful but show it anyhow.
    lower_limit_property_ = new FloatProperty(
      "Lower Limit",
      joint->limits->lower,
      "Lower limit of this joint.  (Not editable)",
      robot_element_property_);
    lower_limit_property_->setReadOnly(true);

    upper_limit_property_ = new FloatProperty(
      "Upper Limit",
      joint->limits->upper,
      "Upper limit of this joint.  (Not editable)",
      robot_element_property_);
    upper_limit_property_->setReadOnly(true);
  }
}

void RobotJoint::showAxisForMovingJoints(
  const urdf::JointConstSharedPtr & joint,
  const std::string & type)
{
  if ((type == "continuous") || (type == "revolute") ||
    (type == "prismatic") || (type == "planar"))
  {
    show_axis_property_ = new Property(
      "Show Joint Axis",
      false,
      "Enable/disable showing the axis of this joint.",
      robot_element_property_,
      SLOT(updateAxis()),
      this);

    axis_property_ = new VectorProperty(
      "Joint Axis",
      Ogre::Vector3(joint->axis.x, joint->axis.y, joint->axis.z),
      "Axis of this joint.  (Not editable)",
      robot_element_property_);
    axis_property_->setReadOnly(true);
  }
}

}  // namespace robot
}  // namespace rviz_default_plugins
