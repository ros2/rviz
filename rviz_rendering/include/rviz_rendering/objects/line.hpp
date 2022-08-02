/*
 * Copyright (c) 2013, Willow Garage, Inc.
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

#ifndef RVIZ_RENDERING__OBJECTS__LINE_HPP_
#define RVIZ_RENDERING__OBJECTS__LINE_HPP_

#include <OgreSceneNode.h>
#include <OgreMaterial.h>
#include <OgreSharedPtr.h>
#include <OgreVector.h>

#include "rviz_rendering/objects/object.hpp"
#include "rviz_rendering/visibility_control.hpp"

namespace Ogre
{
class SceneManager;
class SceneNode;
class Quaternion;
class Any;
class ColourValue;
}

namespace rviz_rendering
{

/* Represents a straight wireframe line between two points. */
class Line : public Object
{
public:
  /**
   * \brief Constructor
   * @param manager Scene manager this object is a part of
   * @param parent_node A scene node to use as the parent of this object. Uses the root scene
   * node if null.
   */
  RVIZ_RENDERING_PUBLIC
  explicit Line(Ogre::SceneManager * manager, Ogre::SceneNode * parent_node = nullptr);

  RVIZ_RENDERING_PUBLIC
  virtual ~Line();

  /**
   * \brief Set the start and end point of the line.
   * @param start The start point.
   * @param end The end point.
   */
  RVIZ_RENDERING_PUBLIC
  void setPoints(Ogre::Vector3 start, Ogre::Vector3 end);

  RVIZ_RENDERING_PUBLIC
  void setVisible(bool visible);

  /**
   * \brief Set the position of this object
   * @param Position vector position to set to.
   */
  RVIZ_RENDERING_PUBLIC
  void setPosition(const Ogre::Vector3 & position) override;

  /**
   * \brief Set the orientation of the object
   * @param Orientation quaternion orientation to set to.
   */
  RVIZ_RENDERING_PUBLIC
  void setOrientation(const Ogre::Quaternion & orientation) override;

  /**
   * \brief Set the scale of the object.  Always relative to the identity orientation of the object.
   * @param Scale vector scale to set to.
   */
  RVIZ_RENDERING_PUBLIC
  void setScale(const Ogre::Vector3 & scale) override;

  /**
   * \brief Set the color of the object.  Values are in the range [0, 1]
   * @param r Red component
   * @param g Green component
   * @param b Blue component
   */
  RVIZ_RENDERING_PUBLIC
  void setColor(float r, float g, float b, float a) override;

  /**
   * \brief Set the color of the object using ogre colour definitions.
   *
   * @param c : ogre colour type.
   */
  RVIZ_RENDERING_PUBLIC
  virtual void setColor(const Ogre::ColourValue & c);

  /**
   * \brief Get the local position of this object
   * @return The position
   */
  RVIZ_RENDERING_PUBLIC
  const Ogre::Vector3 & getPosition() override;

  /**
   * \brief Get the local orientation of this object
   * @return The orientation
   */
  RVIZ_RENDERING_PUBLIC
  const Ogre::Quaternion & getOrientation() override;

  /**
   * \brief Set the user data on this object
   * @param data
   */
  RVIZ_RENDERING_PUBLIC
  void setUserData(const Ogre::Any & data) override;

protected:
  Ogre::SceneNode * scene_node_;
  Ogre::ManualObject * manual_object_;
  Ogre::MaterialPtr manual_object_material_;
};

}  // namespace rviz_rendering

#endif  // RVIZ_RENDERING__OBJECTS__LINE_HPP_
