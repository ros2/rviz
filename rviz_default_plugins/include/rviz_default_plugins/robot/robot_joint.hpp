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

#ifndef RVIZ_DEFAULT_PLUGINS__ROBOT__ROBOT_JOINT_HPP_
#define RVIZ_DEFAULT_PLUGINS__ROBOT__ROBOT_JOINT_HPP_

#include <map>
#include <memory>
#include <string>

#ifndef Q_MOC_RUN

#include <OgreVector.h>
#include <OgreQuaternion.h>
#include <OgreAny.h>
#include <OgreMaterial.h>

#endif

#include <QObject>

#include "urdf/model.h"
#include "urdf_model/pose.h"

#include "rviz_rendering/objects/object.hpp"
#include "rviz_common/interaction/forwards.hpp"

#include "rviz_default_plugins/robot/robot_element_base_class.hpp"
#include "rviz_default_plugins/robot/robot_link.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

namespace Ogre
{
class Quaternion;
}  // namespace Ogre

namespace rviz_rendering
{
class Arrow;
}  // namespace rviz_rendering

namespace rviz_common
{

namespace properties
{
class BoolProperty;
class FloatProperty;
class Property;
class QuaternionProperty;
class StringProperty;
class VectorProperty;
}  // namespace properties
}  // namespace rviz_common

namespace rviz_default_plugins
{
namespace robot
{
class Robot;

/**
 * \struct RobotJoint
 * \brief Contains any data we need from a joint in the robot.
 */
class RVIZ_DEFAULT_PLUGINS_PUBLIC RobotJoint : public RobotElementBaseClass
{
  Q_OBJECT

public:
  RobotJoint(Robot * robot, const urdf::JointConstSharedPtr & joint);
  ~RobotJoint() override;

  void setTransforms(
    const Ogre::Vector3 & parent_link_position,
    const Ogre::Quaternion & parent_link_orientation);

  const std::string & getName() const {return name_;}
  const std::string & getParentLinkName() const {return parent_link_name_;}
  const std::string & getChildLinkName() const {return child_link_name_;}
  const rviz_common::properties::Property * getJointProperty() const
  {
    return robot_element_property_;
  }
  rviz_common::properties::Property * getJointProperty() {return robot_element_property_;}
  RobotJoint * getParentJoint();

  bool hasDescendentLinksWithGeometry() const {return has_decendent_links_with_geometry_;}

  // Set the description for the joint.
  // Also sets the checkbox.
  // Also sets has_decendent_links_with_geometry_.
  // Called when the link_tree style changes.
  void setJointPropertyDescription();

  // set checkboxes based on state of descendent link enables
  // Should only be called by Robot::calculateJointCheckboxes()
  void calculateJointCheckboxesRecursive(
    int & links_with_geom,              // returns # of children with geometry
    int & links_with_geom_checked,      // returns # of enabled children with geometry
    int & links_with_geom_unchecked);   // returns # of disabled children with geometry

private Q_SLOTS:
  void updateAxis();
  void updateChildVisibility();

private:
  bool getEnabled() const override;

  // true if displaying in a tree style.  False if list style.
  bool styleIsTree() const;

  // determine the state of child link(s)
  void getChildLinkState(
    int & links_with_geom,              // returns # of children with geometry
    int & links_with_geom_checked,      // returns # of enabled children with geometry
    int & links_with_geom_unchecked,    // returns # of disabled children with geometry
    bool recursive);              // True: all descendant links.
  // False: just single child link.

  // set the value of the enable checkbox without touching child joints/links
  void setJointCheckbox(QVariant val);
  std::string getType(const urdf::JointConstSharedPtr & joint) const;
  void showLimitProperties(const urdf::JointConstSharedPtr & joint);
  void showAxisForMovingJoints(const urdf::JointConstSharedPtr & joint, const std::string & type);

protected:
  std::string parent_link_name_;
  std::string child_link_name_;

  // properties
  // The joint axis if any, as opposed to the frame in which the joint exists above
  rviz_common::properties::VectorProperty * axis_property_;
  rviz_common::properties::Property * show_axis_property_;
  rviz_common::properties::StringProperty * type_property_;
  rviz_common::properties::FloatProperty * lower_limit_property_;
  rviz_common::properties::FloatProperty * upper_limit_property_;

private:
  RobotLink * links_checked_and_unchecked(
    int & links_with_geom_checked,
    int & links_with_geom_unchecked);
  int links_with_geom(
    RobotLink * link, int & links_with_geom_checked,
    int & links_with_geom_unchecked, int n_args, ...);

  Ogre::Vector3 joint_origin_pos_;
  Ogre::Quaternion joint_origin_rot_;
  bool has_decendent_links_with_geometry_;

  bool doing_set_checkbox_;   // prevents updateChildVisibility() from  touching children

  std::unique_ptr<rviz_rendering::Arrow> axis_;
};

}  // namespace robot
}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__ROBOT__ROBOT_JOINT_HPP_
