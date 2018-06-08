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

#include "rviz_default_plugins/robot/robot_element_base_class.hpp"

#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "rviz_default_plugins/robot/robot.hpp"

#include "rviz_common/load_resource.hpp"
#include "rviz_common/properties/quaternion_property.hpp"
#include "rviz_common/properties/vector_property.hpp"
#include "rviz_rendering/objects/axes.hpp"

using rviz_rendering::Axes;

namespace rviz_default_plugins
{
namespace robot
{

using rviz_common::properties::Property;
using rviz_common::properties::VectorProperty;
using rviz_common::properties::QuaternionProperty;

RobotElementBaseClass::RobotElementBaseClass(Robot * robot, std::string name)
: robot_(robot), name_(std::move(name)) {}

void RobotElementBaseClass::expandDetails(bool expand)
{
  Property * parent = details_->getParent() ? details_ : robot_element_property_;
  if (expand) {
    parent->expand();
  } else {
    parent->collapse();
  }
}

// if use_detail:
//    - all sub properties become children of details_ property.
//    - details_ property becomes a child of robot_element_property_
// else (!use_detail)
//    - all sub properties become children of robot_element_property_.
//    details_ property does not have a parent.
void RobotElementBaseClass::useDetailProperty(bool use_detail)
{
  Property * old_parent = details_->getParent();
  if (old_parent) {
    old_parent->takeChild(details_);
  }

  if (use_detail) {
    while (robot_element_property_->numChildren() > 0) {
      Property * child = robot_element_property_->childAt(0);
      robot_element_property_->takeChild(child);
      details_->addChild(child);
    }

    robot_element_property_->addChild(details_);
  } else {
    while (details_->numChildren() > 0) {
      Property * child = details_->childAt(0);
      details_->takeChild(child);
      robot_element_property_->addChild(child);
    }
  }
}

void RobotElementBaseClass::setParentProperty(rviz_common::properties::Property * new_parent)
{
  Property * old_parent = robot_element_property_->getParent();
  if (old_parent) {
    old_parent->takeChild(robot_element_property_);
  }

  if (new_parent) {
    new_parent->addChild(robot_element_property_);
  }
}

Ogre::Vector3 RobotElementBaseClass::getPosition()
{
  return position_property_->getVector();
}

Ogre::Quaternion RobotElementBaseClass::getOrientation()
{
  return orientation_property_->getQuaternion();
}

void RobotElementBaseClass::updateAxes()
{
  if (axes_property_->getValue().toBool()) {
    if (!axes_) {
      axes_ = std::make_shared<Axes>(
        robot_->getSceneManager(), robot_->getOtherNode(), 0.1f, 0.01f);
      axes_->getSceneNode()->setVisible(getEnabled());

      axes_->setPosition(position_property_->getVector());
      axes_->setOrientation(orientation_property_->getQuaternion());
    }
  } else {
    axes_.reset();
  }
}

}  // namespace robot
}  // namespace rviz_default_plugins
