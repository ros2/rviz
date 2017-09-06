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

#ifndef RVIZ_RENDERING__ORBIT_CAMERA_HPP_
#define RVIZ_RENDERING__ORBIT_CAMERA_HPP_

#include <string>

#include <OgreVector3.h>

#include "rviz_rendering/camera_base.hpp"

namespace Ogre
{
class Camera;
class SceneNode;
class SceneManager;
}

namespace rviz_rendering
{

class Shape;

/// An orbital camera, controlled by yaw, pitch, distance, and focal point.
/**
 * This camera is based on the equation of a sphere in spherical coordinates:
 @verbatim
 x = d*cos(theta)sin(phi)
 y = d*cos(phi)
 z = d*sin(theta)sin(phi)
 @endverbatim
 * Where:<br>
 * d = #distance_<br>
 * theta = #yaw_<br>
 * phi = #pitch_
 */
class OrbitCamera : public CameraBase
{
public:
  explicit OrbitCamera(Ogre::SceneManager * scene_manager);
  virtual ~OrbitCamera();

  /// Move in/out from the focal point, ie. adjust the distance by an amount.
  /**
   * Positive amount moves towards the focal point, negative moves away.
   *
   * \param amount The distance to move.
   */
  void
  zoom(float amount);

  /// Set the focal point of the camera.
  /**
   * Keeps the pitch/yaw/distance the same.
   *
   * \param focal_point The new focal point
   */
  void
  setFocalPoint(const Ogre::Vector3 & focal_point);

  float
  getPitch();  // {return pitch_; }

  float
  getYaw();  // {return yaw_; }

  float
  getDistance();  // {return distance_; }

  const Ogre::Vector3 &
  getFocalPoint();  // {return focal_point_; }

  virtual
  void
  setFrom(CameraBase * camera);

  virtual
  void
  yaw(float angle);

  virtual
  void
  pitch(float angle);

  virtual
  void
  roll(float angle);

  virtual
  void
  setOrientation(float x, float y, float z, float w);

  virtual
  void
  setPosition(float x, float y, float z);

  virtual
  void
  move(float x, float y, float z);  // NOLINT: cpplint thinks this is std::move

  virtual
  Ogre::Vector3
  getPosition();

  virtual
  Ogre::Quaternion
  getOrientation();

  virtual
  void
  lookAt(const Ogre::Vector3 & point);

  virtual
  void
  mouseLeftDrag(int diff_x, int diff_y, bool ctrl, bool alt, bool shift);

  virtual
  void
  mouseMiddleDrag(int diff_x, int diff_y, bool ctrl, bool alt, bool shift);

  virtual
  void
  mouseRightDrag(int diff_x, int diff_y, bool ctrl, bool alt, bool shift);

  virtual
  void
  scrollWheel(int diff, bool ctrl, bool alt, bool shift);

  /// Calculates the camera's position and orientation from yaw, pitch, distance, and focal point.
  virtual
  void
  update();

  virtual
  void
  mouseLeftDown(int x, int y);

  virtual
  void
  mouseMiddleDown(int x, int y);

  virtual
  void
  mouseRightDown(int x, int y);

  virtual
  void
  mouseLeftUp(int x, int y);

  virtual
  void
  mouseMiddleUp(int x, int y);

  virtual
  void
  mouseRightUp(int x, int y);

  virtual
  void
  fromString(const std::string & str);

  virtual
  std::string
  toString();

private:
  Ogre::Vector3
  getGlobalFocalPoint();

  /// Calculate pitch and yaw values given a new position and the current focal point.
  /**
   * \param position Position to calculate the pitch/yaw for
   */
  void
  calculatePitchYawFromPosition(const Ogre::Vector3 & position);

  /// Normalize the camera's pitch, preventing it from reaching vertical (or turning upside down).
  void
  normalizePitch();

  /// Normalize the camera's yaw in the range [0, 2*pi).
  void
  normalizeYaw();

  Ogre::Vector3 focal_point_;  ///< The camera's focal point
  float yaw_;  ///< The camera's yaw (rotation around the y-axis), in radians
  float pitch_;  ///< The camera's pitch (rotation around the x-axis), in radians
  float distance_;  ///< The camera's distance from the focal point

  Shape * focal_point_object_;
};

}  // namespace rviz_rendering

#endif  // RVIZ_RENDERING__ORBIT_CAMERA_HPP_
