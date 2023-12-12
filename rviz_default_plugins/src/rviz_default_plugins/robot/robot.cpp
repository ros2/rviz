/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

#include "rviz_default_plugins/robot/robot.hpp"

#include <map>
#include <string>
#include <vector>

#include <OgreEntity.h>
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreMaterialManager.h>

#include "urdf_model/model.h"

#include "rviz_default_plugins/robot/robot_link.hpp"
#include "rviz_default_plugins/robot/robot_joint.hpp"

#include "rviz_common/properties/property.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/logging.hpp"

#include "rviz_rendering/objects/object.hpp"
#include "rviz_rendering/objects/shape.hpp"
#include "rviz_rendering/objects/axes.hpp"

namespace rviz_default_plugins
{
namespace robot
{
using rviz_common::properties::BoolProperty;
using rviz_common::properties::EnumProperty;
using rviz_common::properties::Property;

Robot::Robot(
  Ogre::SceneNode * root_node, rviz_common::DisplayContext * context, const std::string & name,
  Property * parent_property)
: scene_manager_(context->getSceneManager() ),
  visible_(true),
  visual_visible_(true),
  collision_visible_(false),
  mass_visible_(false),
  inertia_visible_(false),
  context_(context),
  doing_set_checkbox_(false),
  robot_loaded_(false),
  in_changed_enable_all_links_(false),
  name_(name)
{
  root_visual_node_ = root_node->createChildSceneNode();
  root_collision_node_ = root_node->createChildSceneNode();
  root_other_node_ = root_node->createChildSceneNode();

  link_factory_ = new LinkFactory();

  setVisualVisible(visual_visible_);
  setCollisionVisible(collision_visible_);
  setMassVisible(mass_visible_);
  setInertiaVisible(inertia_visible_);
  setAlpha(1.0f);

  link_tree_ = new Property("Links", QVariant(), "", parent_property);
  link_tree_->hide();  // hide until loaded

  link_tree_style_ = new EnumProperty(
    "Link Tree Style",
    "",
    "How the list of links is displayed",
    link_tree_,
    SLOT(changedLinkTreeStyle()),
    this);
  initLinkTreeStyle();
  expand_tree_ = new BoolProperty(
    "Expand Tree",
    false,
    "Expand or collapse link tree",
    link_tree_,
    SLOT(changedExpandTree()),
    this);
  expand_link_details_ = new BoolProperty(
    "Expand Link Details",
    false,
    "Expand link details (sub properties) to see all info for all links.",
    link_tree_,
    SLOT(changedExpandLinkDetails()),
    this);
  expand_joint_details_ = new BoolProperty(
    "Expand Joint Details",
    false,
    "Expand joint details (sub properties) to see all info for all joints.",
    link_tree_,
    SLOT(changedExpandJointDetails()),
    this);
  enable_all_links_ = new BoolProperty(
    "All Links Enabled",
    true,
    "Turn all links on or off.",
    link_tree_,
    SLOT(changedEnableAllLinks()),
    this);
}

Robot::~Robot()
{
  clear();

  scene_manager_->destroySceneNode(root_visual_node_);
  scene_manager_->destroySceneNode(root_collision_node_);
  scene_manager_->destroySceneNode(root_other_node_);
  delete link_factory_;
}

void Robot::load(
  const urdf::ModelInterface & urdf,
  bool visual,
  bool collision,
  bool mass,
  bool inertia)
{
  link_tree_->hide();  // hide until loaded
  robot_loaded_ = false;

  // clear out any data (properties, shapes, etc) from a previously loaded robot.
  clear();

  // the root link is discovered below.  Set to nullptr until found.
  root_link_ = nullptr;

  // Properties are not added to display until changedLinkTreeStyle() is called (below).
  createLinkProperties(urdf, visual, collision, mass, inertia);
  createJointProperties(urdf);

  // robot is now loaded
  robot_loaded_ = true;
  link_tree_->show();

  // set the link tree style and add link/joint properties to rviz pane.
  setLinkTreeStyle(LinkTreeStyle(link_tree_style_->getOptionInt()));
  changedLinkTreeStyle();

  // at startup the link tree is collapsed since it is large and not often needed.
  link_tree_->collapse();

  setVisualVisible(isVisualVisible() );
  setCollisionVisible(isCollisionVisible() );
  setMassVisible(isMassVisible());
  setInertiaVisible(isInertiaVisible());
}

void Robot::clear()
{
  // unparent all link and joint properties so they can be deleted in arbitrary order
  // without being delete by their parent propeties (which vary based on style)
  unparentLinkProperties();

  for (const auto & link_entry : links_) {
    RobotLink * link = link_entry.second;
    delete link;
  }

  for (const auto & joint_entry : joints_) {
    RobotJoint * joint = joint_entry.second;
    delete joint;
  }

  links_.clear();
  joints_.clear();
  root_visual_node_->removeAndDestroyAllChildren();
  root_collision_node_->removeAndDestroyAllChildren();
  root_other_node_->removeAndDestroyAllChildren();
}

void Robot::update(const LinkUpdater & updater)
{
  for (const auto & link_entry : links_) {
    RobotLink * link = link_entry.second;

    link->setToNormalMaterial();

    Ogre::Vector3 visual_position, collision_position;
    Ogre::Quaternion visual_orientation, collision_orientation;
    if (updater.getLinkTransforms(
        link->getName(),
        visual_position,
        visual_orientation,
        collision_position,
        collision_orientation))
    {
      // Check if visual_orientation, visual_position, collision_orientation,
      // and collision_position are NaN.
      if (visual_orientation.isNaN()) {
        log_error(link, "visual", "orientation");
        continue;
      }
      if (visual_position.isNaN()) {
        log_error(link, "visual", "position");
        continue;
      }
      if (collision_orientation.isNaN()) {
        log_error(link, "collision", "orientation");
        continue;
      }
      if (collision_position.isNaN()) {
        log_error(link, "collision", "position");
        continue;
      }
      link->setTransforms(
        visual_position, visual_orientation, collision_position,
        collision_orientation);

      for (const auto & child_joint_name : link->getChildJointNames()) {
        RobotJoint * joint = getJoint(child_joint_name);
        if (joint) {
          joint->setTransforms(visual_position, visual_orientation);
        }
      }
    } else {
      link->setToErrorMaterial();
    }
  }
}

void Robot::log_error(
  const RobotLink * link,
  const std::string & visual_or_collision,
  const std::string & position_or_orientation) const
{
  // TODO(wjwwood): restore throttle behavior of ROS_ERROR_THROTTLE
  RVIZ_COMMON_LOG_ERROR_STREAM(
    // 1.0,
    visual_or_collision << ": " << position_or_orientation << " of link " << link->getName() <<
      " contains NaNs. Skipping render as long as the " << position_or_orientation <<
      " is invalid.";
  );
}

void Robot::setVisible(bool visible)
{
  visible_ = visible;
  if (visible) {
    root_visual_node_->setVisible(visual_visible_);
    root_collision_node_->setVisible(collision_visible_);
    updateLinkVisibilities();
  } else {
    root_visual_node_->setVisible(false);
    root_collision_node_->setVisible(false);
    updateLinkVisibilities();
  }
}

void Robot::setVisualVisible(bool visible)
{
  visual_visible_ = visible;
  updateLinkVisibilities();
}

void Robot::setCollisionVisible(bool visible)
{
  collision_visible_ = visible;
  updateLinkVisibilities();
}

void Robot::setMassVisible(bool visible)
{
  mass_visible_ = visible;
  updateLinkVisibilities();
}

void Robot::setInertiaVisible(bool visible)
{
  inertia_visible_ = visible;
  updateLinkVisibilities();
}

void Robot::updateLinkVisibilities()
{
  for (auto & link_map_entry : links_) {
    RobotLink * link = link_map_entry.second;
    link->updateVisibility();
  }
}

bool Robot::isVisible()
{
  return visible_;
}

bool Robot::isVisualVisible()
{
  return visual_visible_;
}

bool Robot::isCollisionVisible()
{
  return collision_visible_;
}

bool Robot::isMassVisible()
{
  return mass_visible_;
}

bool Robot::isInertiaVisible()
{
  return inertia_visible_;
}

void Robot::setAlpha(float a)
{
  alpha_ = a;

  for (auto & link_map_entry : links_) {
    RobotLink * link = link_map_entry.second;
    link->setRobotAlpha(alpha_);
  }
}

void Robot::setPosition(const Ogre::Vector3 & position)
{
  root_visual_node_->setPosition(position);
  root_collision_node_->setPosition(position);
}

void Robot::setOrientation(const Ogre::Quaternion & orientation)
{
  root_visual_node_->setOrientation(orientation);
  root_collision_node_->setOrientation(orientation);
}

void Robot::setScale(const Ogre::Vector3 & scale)
{
  root_visual_node_->setScale(scale);
  root_collision_node_->setScale(scale);
}

const Ogre::Vector3 & Robot::getPosition()
{
  return root_visual_node_->getPosition();
}

const Ogre::Quaternion & Robot::getOrientation()
{
  return root_visual_node_->getOrientation();
}

void Robot::setLinkTreeStyle(LinkTreeStyle style)
{
  std::map<LinkTreeStyle, std::string>::const_iterator style_it = style_name_map_.find(style);
  if (style_it == style_name_map_.end()) {
    link_tree_style_->setValue(style_name_map_[STYLE_DEFAULT].c_str());
  } else {
    link_tree_style_->setValue(style_it->second.c_str());
  }
}

void Robot::calculateJointCheckboxes()
{
  if (in_changed_enable_all_links_ || !robot_loaded_) {
    return;
  }

  int links_with_geom_checked = 0;
  int links_with_geom_unchecked = 0;

  // check root link
  RobotLink * link = root_link_;

  if (!link) {
    setEnableAllLinksCheckbox(QVariant());
    return;
  }

  if (link->hasGeometry()) {
    bool checked = link->getLinkProperty()->getValue().toBool();
    links_with_geom_checked += checked ? 1 : 0;
    links_with_geom_unchecked += checked ? 0 : 1;
  }

  // check all child links and joints recursively
  for (auto & child_joint_name : link->getChildJointNames()) {
    RobotJoint * child_joint = getJoint(child_joint_name);
    if (child_joint) {
      int child_links_with_geom;
      int child_links_with_geom_checked;
      int child_links_with_geom_unchecked;
      child_joint->calculateJointCheckboxesRecursive(
        child_links_with_geom,
        child_links_with_geom_checked,
        child_links_with_geom_unchecked);
      links_with_geom_checked += child_links_with_geom_checked;
      links_with_geom_unchecked += child_links_with_geom_unchecked;
    }
  }
  int links_with_geom = links_with_geom_checked + links_with_geom_unchecked;

  if (!links_with_geom) {
    setEnableAllLinksCheckbox(QVariant());
  } else {
    setEnableAllLinksCheckbox(links_with_geom_unchecked == 0);
  }
}

void Robot::setLinkFactory(LinkFactory * link_factory)
{
  if (link_factory) {
    delete link_factory_;
    link_factory_ = link_factory;
  }
}

RobotLink * Robot::LinkFactory::createLink(
  Robot * robot,
  const urdf::LinkConstSharedPtr & link,
  const std::string & parent_joint_name,
  bool visual,
  bool collision,
  bool mass,
  bool inertia)
{
  return new RobotLink(robot, link, parent_joint_name, visual, collision, mass, inertia);
}

RobotJoint * Robot::LinkFactory::createJoint(
  Robot * robot,
  const urdf::JointConstSharedPtr & joint)
{
  return new RobotJoint(robot, joint);
}

void Robot::createLinkProperties(
  const urdf::ModelInterface & urdf,
  bool visual,
  bool collision,
  bool mass,
  bool inertia)
{
  for (const auto & link_entry : urdf.links_) {
    const urdf::LinkConstSharedPtr & urdf_link = link_entry.second;
    std::string parent_joint_name;

    if (urdf_link != urdf.getRoot() && urdf_link->parent_joint) {
      parent_joint_name = urdf_link->parent_joint->name;
    }

    RobotLink * link = link_factory_->createLink(
      this, urdf_link, parent_joint_name, visual, collision, mass, inertia);

    if (urdf_link == urdf.getRoot()) {
      root_link_ = link;
    }

    links_[urdf_link->name] = link;

    link->setRobotAlpha(alpha_);
  }
}

void Robot::createJointProperties(const urdf::ModelInterface & urdf)
{
  for (const auto & joint_entry : urdf.joints_) {
    const urdf::JointConstSharedPtr & urdf_joint = joint_entry.second;
    RobotJoint * joint = link_factory_->createJoint(this, urdf_joint);

    joints_[urdf_joint->name] = joint;
  }
}

void Robot::unparentLinkProperties()
{
  for (const auto & link : links_) {
    link.second->setParentProperty(nullptr);
  }

  for (const auto & joint : joints_) {
    joint.second->setParentProperty(nullptr);
  }
}

void Robot::useDetailProperty(bool use_detail)
{
  for (auto & link : links_) {
    link.second->useDetailProperty(use_detail);
  }

  for (auto & joint : joints_) {
    joint.second->useDetailProperty(use_detail);
  }
}

void Robot::changedExpandTree()
{
  bool expand = expand_tree_->getBool();

  for (auto & link : links_) {
    if (expand) {
      link.second->getLinkProperty()->expand();
    } else {
      link.second->getLinkProperty()->collapse();
    }
  }

  for (auto & joint : joints_) {
    if (expand) {
      joint.second->getJointProperty()->expand();
    } else {
      joint.second->getJointProperty()->collapse();
    }
  }
}

void Robot::changedExpandLinkDetails()
{
  bool expand = expand_link_details_->getBool();

  for (auto & link : links_) {
    link.second->expandDetails(expand);
  }
}

void Robot::changedExpandJointDetails()
{
  bool expand = expand_joint_details_->getBool();

  for (auto & joint : joints_) {
    joint.second->expandDetails(expand);
  }
}

void Robot::changedEnableAllLinks()
{
  if (doing_set_checkbox_) {
    return;
  }

  bool enable = enable_all_links_->getBool();

  in_changed_enable_all_links_ = true;

  for (auto & link : links_) {
    if (link.second->hasGeometry()) {
      link.second->getLinkProperty()->setValue(enable);
    }
  }

  for (auto & joint : joints_) {
    if (joint.second->hasDescendentLinksWithGeometry()) {
      joint.second->getJointProperty()->setValue(enable);
    }
  }

  in_changed_enable_all_links_ = false;
}

void Robot::setEnableAllLinksCheckbox(QVariant val)
{
  // doing_set_checkbox_ prevents changedEnableAllLinks from turning all
  // links off when we modify the enable_all_links_ property.
  doing_set_checkbox_ = true;
  enable_all_links_->setValue(val);
  doing_set_checkbox_ = false;
}

void Robot::initLinkTreeStyle()
{
  style_name_map_.clear();
  style_name_map_[STYLE_LINK_LIST] = "Links in Alphabetic Order";
  style_name_map_[STYLE_JOINT_LIST] = "Joints in Alphabetic Order";
  style_name_map_[STYLE_LINK_TREE] = "Tree of links";
  style_name_map_[STYLE_JOINT_LINK_TREE] = "Tree of links and joints";

  link_tree_style_->clearOptions();
  for (auto & style : style_name_map_) {
    link_tree_style_->addOptionStd(style.second, style.first);
  }
}

// insert properties into link_tree_ according to style
void Robot::changedLinkTreeStyle()
{
  if (!robot_loaded_) {
    return;
  }

  auto style = LinkTreeStyle(link_tree_style_->getOptionInt());

  unparentLinkProperties();

  switch (style) {
    case STYLE_LINK_TREE:
    case STYLE_JOINT_LINK_TREE:
      useDetailProperty(true);
      if (root_link_) {
        addLinkToLinkTree(style, link_tree_, root_link_);
      }
      break;

    case STYLE_JOINT_LIST:
      {
        useDetailProperty(false);
        for (const auto & joint_entry : joints_) {
          joint_entry.second->setParentProperty(link_tree_);
          joint_entry.second->setJointPropertyDescription();
        }
        break;
      }

    case STYLE_LINK_LIST:
    default:
      useDetailProperty(false);
      for (const auto & link_entry : links_) {
        link_entry.second->setParentProperty(link_tree_);
      }
      break;
  }

  switch (style) {
    case STYLE_LINK_TREE:
      link_tree_->setName("Link Tree");
      link_tree_->setDescription(
        "A tree of all links in the robot.  Uncheck a link to hide its geometry.");
      expand_tree_->show();
      expand_link_details_->show();
      expand_joint_details_->hide();
      break;
    case STYLE_JOINT_LINK_TREE:
      link_tree_->setName("Link/Joint Tree");
      link_tree_->setDescription(
        "A tree of all joints and links in the robot.  Uncheck a link to hide its geometry.");
      expand_tree_->show();
      expand_link_details_->show();
      expand_joint_details_->show();
      break;
    case STYLE_JOINT_LIST:
      link_tree_->setName("Joints");
      link_tree_->setDescription("All joints in the robot in alphabetic order.");
      expand_tree_->hide();
      expand_link_details_->hide();
      expand_joint_details_->show();
      break;
    case STYLE_LINK_LIST:
    default:
      link_tree_->setName("Links");
      link_tree_->setDescription(
        "All links in the robot in alphabetic order.  Uncheck a link to hide its geometry.");
      expand_tree_->hide();
      expand_link_details_->show();
      expand_joint_details_->hide();
      break;
  }

  expand_link_details_->setValue(false);
  expand_joint_details_->setValue(false);
  expand_tree_->setValue(false);
  calculateJointCheckboxes();
}

// recursive helper for setLinkTreeStyle() when style is *_TREE
void Robot::addLinkToLinkTree(LinkTreeStyle style, Property * parent, RobotLink * link)
{
  if (styleShowLink(style)) {
    link->setParentProperty(parent);
    parent = link->getLinkProperty();
  }

  for (const auto & child_joint_name : link->getChildJointNames()) {
    RobotJoint * child_joint = getJoint(child_joint_name);
    if (child_joint) {
      addJointToLinkTree(style, parent, child_joint);
    }
  }
}

// recursive helper for setLinkTreeStyle() when style is *_TREE
void Robot::addJointToLinkTree(LinkTreeStyle style, Property * parent, RobotJoint * joint)
{
  if (styleShowJoint(style)) {
    joint->setParentProperty(parent);
    parent = joint->getJointProperty();
    joint->setJointPropertyDescription();
  }

  RobotLink * link = getLink(joint->getChildLinkName() );
  if (link) {
    addLinkToLinkTree(style, parent, link);
  }
}

RobotLink * Robot::getLink(const std::string & name)
{
  auto it = links_.find(name);
  if (it == links_.end() ) {
    RVIZ_COMMON_LOG_WARNING_STREAM(
      "Link [" << name.c_str() << "] does not exist");
    return nullptr;
  }

  return it->second;
}

RobotJoint * Robot::getJoint(const std::string & name)
{
  auto it = joints_.find(name);
  if (it == joints_.end() ) {
    RVIZ_COMMON_LOG_WARNING_STREAM(
      "Joint [" << name.c_str() << "] does not exist");
    return nullptr;
  }

  return it->second;
}

bool Robot::styleShowLink(LinkTreeStyle style)
{
  return
    style == STYLE_LINK_LIST ||
    style == STYLE_LINK_TREE ||
    style == STYLE_JOINT_LINK_TREE;
}

bool Robot::styleShowJoint(LinkTreeStyle style)
{
  return
    style == STYLE_JOINT_LIST ||
    style == STYLE_JOINT_LINK_TREE;
}

bool Robot::styleIsTree(LinkTreeStyle style)
{
  return
    style == STYLE_LINK_TREE ||
    style == STYLE_JOINT_LINK_TREE;
}

}  // namespace robot
}  // namespace rviz_default_plugins
