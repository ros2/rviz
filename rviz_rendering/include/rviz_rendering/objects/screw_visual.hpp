/*
 * Copyright (c) 2023, Open Source Robotics Foundation, Inc.
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
#ifndef RVIZ_RENDERING__OBJECTS__SCREW_VISUAL_HPP_
#define RVIZ_RENDERING__OBJECTS__SCREW_VISUAL_HPP_

#include "rviz_rendering/objects/screw_visual.hpp"

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include <memory>

#include "rviz_rendering/objects/arrow.hpp"
#include "rviz_rendering/objects/billboard_line.hpp"

#include "rviz_rendering/visibility_control.hpp"

namespace rviz_rendering
{
// ScrewVisual visualizes a single screw, i.e. a wrench, twist, or acceleration
class ScrewVisual
{
public:
  // Constructor.
  // Creates the visual stuff and puts it into the scene, but in an unconfigured state.
  RVIZ_RENDERING_PUBLIC
  ScrewVisual(Ogre::SceneManager * scene_manager, Ogre::SceneNode * parent_node);

  // Destructor.  Removes the visual stuff from the scene.
  virtual ~ScrewVisual();

  // Configure the visual to show the given linear and angular vectors
  RVIZ_RENDERING_PUBLIC
  void setScrew(const Ogre::Vector3 & linear, const Ogre::Vector3 & angular);

  // Set the pose of the coordinate frame the message refers to.
  // This could be done inside setMessage(), but that would require calls to FrameManager
  // and error handling inside setMessage(), which doesn't seem as clean.
  // This way ScrewVisual is only responsible for visualization.
  RVIZ_RENDERING_PUBLIC
  void setFramePosition(const Ogre::Vector3 & position);
  RVIZ_RENDERING_PUBLIC
  void setFrameOrientation(const Ogre::Quaternion & orientation);

  // Set the color and alpha of the visual, which are user-editable
  // parameters and therefore don't come from the message.
  RVIZ_RENDERING_PUBLIC
  void setLinearColor(float r, float g, float b, float a);
  RVIZ_RENDERING_PUBLIC
  void setAngularColor(float r, float g, float b, float a);
  RVIZ_RENDERING_PUBLIC
  void setLinearScale(float s);
  RVIZ_RENDERING_PUBLIC
  void setAngularScale(float s);
  RVIZ_RENDERING_PUBLIC
  void setWidth(float w);
  RVIZ_RENDERING_PUBLIC
  void setHideSmallValues(bool h);
  RVIZ_RENDERING_PUBLIC
  void setVisible(bool visible);

private:
  // The object implementing the circle
  std::unique_ptr<rviz_rendering::Arrow> arrow_linear_;
  std::unique_ptr<rviz_rendering::Arrow> arrow_angular_;
  std::unique_ptr<rviz_rendering::BillboardLine> circle_angular_;
  std::unique_ptr<rviz_rendering::Arrow> circle_arrow_angular_;
  float linear_scale_;
  float angular_scale_;
  float width_;
  bool hide_small_values_;

  // A SceneNode whose pose is set to match the coordinate frame of the message header.
  Ogre::SceneNode * frame_node_;
  // allow showing/hiding of linear / angular arrows
  Ogre::SceneNode * linear_node_;
  Ogre::SceneNode * angular_node_;

  // The SceneManager, kept here only so the destructor can ask it to destroy the ``frame_node_``.
  Ogre::SceneManager * scene_manager_;
};
}  // namespace rviz_rendering

#endif  // RVIZ_RENDERING__OBJECTS__SCREW_VISUAL_HPP_
