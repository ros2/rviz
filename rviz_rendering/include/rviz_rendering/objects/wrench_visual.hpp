/*
 * Copyright (c) 2019, Martin Idel and others
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

#ifndef RVIZ_RENDERING__OBJECTS__WRENCH_VISUAL_HPP_
#define RVIZ_RENDERING__OBJECTS__WRENCH_VISUAL_HPP_

#include <memory>

#include <OgreVector.h>
#include <OgreQuaternion.h>

#include "rviz_rendering/visibility_control.hpp"

namespace rviz_rendering
{
class Arrow;
class BillboardLine;
}

namespace rviz_rendering
{

/*
 * Each instance of WrenchVisual represents the visualization of a single
 * sensor_msgs::msg::WrenchStamped message.
 */
class WrenchVisual
{
public:
  RVIZ_RENDERING_PUBLIC
  WrenchVisual(Ogre::SceneManager * scene_manager, Ogre::SceneNode * parent_node);

  RVIZ_RENDERING_PUBLIC
  virtual ~WrenchVisual();

  // Configure the visual to show the given force and torque vectors
  RVIZ_RENDERING_PUBLIC
  void setWrench(const Ogre::Vector3 & force, const Ogre::Vector3 & torque);

  // Set the pose of the coordinate frame the message refers to.
  RVIZ_RENDERING_PUBLIC
  void setFramePosition(const Ogre::Vector3 & position);

  RVIZ_RENDERING_PUBLIC
  void setFrameOrientation(const Ogre::Quaternion & orientation);

  RVIZ_RENDERING_PUBLIC
  void setForceColor(float r, float g, float b, float a);

  RVIZ_RENDERING_PUBLIC
  void setTorqueColor(float r, float g, float b, float a);

  RVIZ_RENDERING_PUBLIC
  void setForceScale(float scale);

  RVIZ_RENDERING_PUBLIC
  void setTorqueScale(float scale);

  RVIZ_RENDERING_PUBLIC
  void setWidth(float width);

  RVIZ_RENDERING_PUBLIC
  void setVisible(bool visible);

private:
  void createTorqueDirectionCircle(const Ogre::Quaternion & orientation) const;
  void setTorqueDirectionArrow(const Ogre::Quaternion & orientation) const;
  Ogre::Quaternion getDirectionOfRotationRelativeToTorque(
    const Ogre::Vector3 & torque, const Ogre::Vector3 & axis_z) const;
  void updateForceArrow() const;
  void updateTorque() const;

  std::shared_ptr<rviz_rendering::Arrow> arrow_force_;
  std::shared_ptr<rviz_rendering::Arrow> arrow_torque_;
  std::shared_ptr<rviz_rendering::BillboardLine> circle_torque_;
  std::shared_ptr<rviz_rendering::Arrow> circle_arrow_torque_;
  Ogre::Vector3 force_arrow_direction_;
  Ogre::Vector3 torque_arrow_direction_;
  float force_scale_;
  float torque_scale_;
  float width_;

  Ogre::SceneNode * frame_node_;
  Ogre::SceneNode * force_node_;
  Ogre::SceneNode * torque_node_;
  Ogre::SceneManager * scene_manager_;
};

}  // namespace rviz_rendering

#endif  // RVIZ_RENDERING__OBJECTS__WRENCH_VISUAL_HPP_
