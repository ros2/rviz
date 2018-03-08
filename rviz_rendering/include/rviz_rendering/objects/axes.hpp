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

#ifndef RVIZ_RENDERING__OBJECTS__AXES_HPP_
#define RVIZ_RENDERING__OBJECTS__AXES_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

#include "object.hpp"
#include "rviz_rendering/visibility_control.hpp"

namespace Ogre
{
class SceneManager;
class SceneNode;
class Vector3;
class Quaternion;
class Any;
class ColourValue;
}

namespace rviz_rendering
{
class Shape;

/**
 * \class Axes
 * \brief An object that displays a set of X/Y/Z axes, with X=Red, Y=Green, Z=Blue
 */
class Axes : public Object
{
public:
  /**
   * \brief Constructor
   * @param manager Scene manager this object is a part of
   * @param parent_node A scene node to use as the parent of this object.  If NULL, uses the root scene node.
   * @param length Length of the axes
   * @param radius Radius of the axes
   */
  RVIZ_RENDERING_PUBLIC
  Axes(
    Ogre::SceneManager * manager, Ogre::SceneNode * parent_node = NULL, float length = 1.0f,
    float radius = 0.1f);
  virtual ~Axes();

  /**
   * \brief Set the parameters on this object
   *
   * @param length Length of the axes
   * @param radius Radius of the axes
   */
  RVIZ_RENDERING_PUBLIC
  void set(float length, float radius);

  RVIZ_RENDERING_PUBLIC
  virtual void setOrientation(const Ogre::Quaternion & orientation);

  RVIZ_RENDERING_PUBLIC
  virtual void setPosition(const Ogre::Vector3 & position);

  RVIZ_RENDERING_PUBLIC
  virtual void setScale(const Ogre::Vector3 & scale);

  RVIZ_RENDERING_PUBLIC
  virtual void setColor(float r, float g, float b, float a);

  RVIZ_RENDERING_PUBLIC
  virtual const Ogre::Vector3 & getPosition();

  RVIZ_RENDERING_PUBLIC
  virtual const Ogre::Quaternion & getOrientation();

  /**
   * \brief Get the scene node associated with this object
   * @return The scene node associated with this object
   */
  Ogre::SceneNode * getSceneNode() {return scene_node_;}

  /**
   * \brief Sets user data on all ogre objects we own
   */
  RVIZ_RENDERING_PUBLIC
  void setUserData(const Ogre::Any & data);

  RVIZ_RENDERING_PUBLIC
  Shape * getXShape() {return x_axis_;}

  RVIZ_RENDERING_PUBLIC
  Shape * getYShape() {return y_axis_;}

  RVIZ_RENDERING_PUBLIC
  Shape * getZShape() {return z_axis_;}

  RVIZ_RENDERING_PUBLIC
  void setXColor(const Ogre::ColourValue & col);

  RVIZ_RENDERING_PUBLIC
  void setYColor(const Ogre::ColourValue & col);

  RVIZ_RENDERING_PUBLIC
  void setZColor(const Ogre::ColourValue & col);

  RVIZ_RENDERING_PUBLIC
  void setToDefaultColors();

  RVIZ_RENDERING_PUBLIC
  static const Ogre::ColourValue & getDefaultXColor();

  RVIZ_RENDERING_PUBLIC
  static const Ogre::ColourValue & getDefaultYColor();

  RVIZ_RENDERING_PUBLIC
  static const Ogre::ColourValue & getDefaultZColor();

private:
  // prohibit copying
  Axes(const Axes & other)
  : Object(0) {(void) other;}
  Axes & operator=(const Axes & other) {(void) other; return *this;}

  Ogre::SceneNode * scene_node_;

  Shape * x_axis_;      ///< Cylinder for the X-axis
  Shape * y_axis_;      ///< Cylinder for the Y-axis
  Shape * z_axis_;      ///< Cylinder for the Z-axis

  static const Ogre::ColourValue default_x_color_;
  static const Ogre::ColourValue default_y_color_;
  static const Ogre::ColourValue default_z_color_;
};

}  // namespace rviz_rendering

#endif  // RVIZ_RENDERING__OBJECTS__AXES_HPP_
