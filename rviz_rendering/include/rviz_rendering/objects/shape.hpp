/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * Copyright (c) 2017, Open Source Robotics Foundation, Inc.
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

#ifndef RVIZ_RENDERING__OBJECTS__SHAPE_HPP_
#define RVIZ_RENDERING__OBJECTS__SHAPE_HPP_

#include <string>

#include <OgreMaterial.h>
#include <OgreSharedPtr.h>
#include <OgreVector.h>

#include "rviz_rendering/objects/object.hpp"
#include "rviz_rendering/visibility_control.hpp"

namespace Ogre
{
class SceneManager;
class SceneNode;
class Any;
class Entity;
}

namespace rviz_rendering
{

class Shape : public Object
{
public:
  enum Type
  {
    Cone,
    Cube,
    Cylinder,
    Sphere,
    Mesh,
  };

  /**
   * \brief Constructor
   *
   * @param scene_manager The scene manager this object is associated with
   * @param parent_node A scene node to use as the parent of this object.  If NULL, uses the root scene node.
   */
  RVIZ_RENDERING_PUBLIC
  Shape(Type shape_type, Ogre::SceneManager * scene_manager, Ogre::SceneNode * parent_node = NULL);

  RVIZ_RENDERING_PUBLIC
  virtual ~Shape();

  RVIZ_RENDERING_PUBLIC
  Type getType() {return type_;}

  /**
   * \brief Set the offset for this shape
   *
   * The default is no offset, which puts the pivot point directly in the center of the object.
   *
   * @param offset Amount to offset the center of the object from the pivot point
   */
  RVIZ_RENDERING_PUBLIC
  void setOffset(const Ogre::Vector3 & offset);

  RVIZ_RENDERING_PUBLIC
  virtual void setColor(float r, float g, float b, float a);

  RVIZ_RENDERING_PUBLIC
  void setColor(const Ogre::ColourValue & c);

  RVIZ_RENDERING_PUBLIC
  virtual void setPosition(const Ogre::Vector3 & position);

  RVIZ_RENDERING_PUBLIC
  virtual void setOrientation(const Ogre::Quaternion & orientation);

  RVIZ_RENDERING_PUBLIC
  virtual void setScale(const Ogre::Vector3 & scale);

  RVIZ_RENDERING_PUBLIC
  virtual const Ogre::Vector3 & getPosition();

  RVIZ_RENDERING_PUBLIC
  virtual const Ogre::Quaternion & getOrientation();

  /**
   * \brief Get the root scene node (pivot node) for this object
   *
   * @return The root scene node of this object
   */
  RVIZ_RENDERING_PUBLIC
  Ogre::SceneNode * getRootNode() {return scene_node_;}

  /**
   * \brief Sets user data on all ogre objects we own
   */
  RVIZ_RENDERING_PUBLIC
  void setUserData(const Ogre::Any & data);

  RVIZ_RENDERING_PUBLIC
  Ogre::Entity * getEntity() {return entity_;}

  RVIZ_RENDERING_PUBLIC
  Ogre::MaterialPtr getMaterial() {return material_;}

  RVIZ_RENDERING_PUBLIC
  static Ogre::Entity * createEntity(
    const std::string & name, Type shape_type,
    Ogre::SceneManager * scene_manager);

protected:
  Ogre::SceneNode * scene_node_;
  Ogre::SceneNode * offset_node_;
  Ogre::Entity * entity_;
  Ogre::MaterialPtr material_;
  std::string material_name_;

  Type type_;
};

}  // namespace rviz_rendering

#endif  // RVIZ_RENDERING__OBJECTS__SHAPE_HPP_
