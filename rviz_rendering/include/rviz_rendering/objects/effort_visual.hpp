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

#ifndef RVIZ_RENDERING__OBJECTS__EFFORT_VISUAL_HPP_
#define RVIZ_RENDERING__OBJECTS__EFFORT_VISUAL_HPP_

#include <map>
#include <memory>
#include <string>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include "rviz_rendering/objects/arrow.hpp"
#include "rviz_rendering/objects/billboard_line.hpp"

#include "rviz_rendering/visibility_control.hpp"

namespace rviz_rendering
{
class EffortVisual
{
public:
  RVIZ_RENDERING_PUBLIC
  EffortVisual(
    Ogre::SceneManager * scene_manager, Ogre::SceneNode * parent_node, float width, float scale);

  // set rainbow color
  RVIZ_RENDERING_PUBLIC
  void getRainbowColor(float value, Ogre::ColourValue & color);
  RVIZ_RENDERING_PUBLIC
  void setEffort(const std::string & joint_name, double effort, double max_effort);

  // set the pose of coordinates frame the each joint refers to.
  RVIZ_RENDERING_PUBLIC
  void setFramePosition(const std::string & joint_name, const Ogre::Vector3 & position);
  RVIZ_RENDERING_PUBLIC
  void setFrameOrientation(const std::string & joint_name, const Ogre::Quaternion & orientation);

  RVIZ_RENDERING_PUBLIC
  void setFrameEnabled(const std::string & joint_name, const bool e);

  RVIZ_RENDERING_PUBLIC
  void setWidth(float w);

  RVIZ_RENDERING_PUBLIC
  void setScale(float s);

private:
  // The object implementing the effort circle
  std::map<std::string, std::unique_ptr<rviz_rendering::BillboardLine>> effort_circle_;
  std::map<std::string, std::unique_ptr<rviz_rendering::Arrow>> effort_arrow_;
  std::map<std::string, bool> effort_enabled_;

  Ogre::SceneManager * scene_manager_;
  Ogre::SceneNode * parent_node_;

  std::map<std::string, Ogre::Vector3> position_;
  std::map<std::string, Ogre::Quaternion> orientation_;

  float width_, scale_;
};
}  // namespace rviz_rendering

#endif  // RVIZ_RENDERING__OBJECTS__EFFORT_VISUAL_HPP_
