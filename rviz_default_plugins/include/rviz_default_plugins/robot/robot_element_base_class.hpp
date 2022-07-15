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

#ifndef RVIZ_DEFAULT_PLUGINS__ROBOT__ROBOT_ELEMENT_BASE_CLASS_HPP_
#define RVIZ_DEFAULT_PLUGINS__ROBOT__ROBOT_ELEMENT_BASE_CLASS_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <OgreVector.h>
#include <OgreQuaternion.h>
#include <OgreSceneNode.h>

#include <QObject>  // NOLINT cpplint cannot handle include order here

#include "urdf/model.h"  // can be replaced later by urdf_model/types.h
#include "urdf_model/pose.h"

#include "rviz_rendering/objects/object.hpp"
#include "rviz_common/interaction/forwards.hpp"

#include "rviz_default_plugins/visibility_control.hpp"

namespace Ogre
{
class Quaternion;
}  // namespace Ogre

namespace rviz_rendering
{
class Axes;
}  // namespace rviz_rendering

namespace rviz_common
{
namespace properties
{
class FloatProperty;
class Property;
class BoolProperty;
class QuaternionProperty;
class VectorProperty;
}  // namespace properties
}  // namespace rviz_common

namespace rviz_default_plugins
{
namespace robot
{

class Robot;

class RVIZ_DEFAULT_PLUGINS_PUBLIC RobotElementBaseClass : public QObject
{
  Q_OBJECT

public:
  RobotElementBaseClass(Robot * robot, std::string name);

  // expand all sub properties
  void expandDetails(bool expand);

  // place subproperties as children of details_ or joint_property_
  void useDetailProperty(bool use_detail);

  // Remove robot_element_property_ from its old parent and add to new_parent.
  // If new_parent==NULL then leav unparented.
  void setParentProperty(rviz_common::properties::Property * new_parent);

  Ogre::Vector3 getPosition();
  Ogre::Quaternion getOrientation();

private Q_SLOTS:
  void updateAxes();

protected:
  Robot * robot_;
  std::string name_;                          ///< Name of this robot part.

  // properties
  rviz_common::properties::Property * robot_element_property_;  // either joint or link property.
  rviz_common::properties::Property * details_;
  rviz_common::properties::VectorProperty * position_property_;
  rviz_common::properties::QuaternionProperty * orientation_property_;
  rviz_common::properties::Property * axes_property_;
  std::shared_ptr<rviz_rendering::Axes> axes_;

private:
  virtual bool getEnabled() const = 0;
};

}  // namespace robot
}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__ROBOT__ROBOT_ELEMENT_BASE_CLASS_HPP_
