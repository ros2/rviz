/*
 * Copyright (c) 2012, Willow Garage, Inc.
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
 *     * Neither the name of the copyright holder nor the names of its contributors
 *       may be used to endorse or promote products derived from
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

#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__MAP__SWATCH_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__MAP__SWATCH_HPP_

#include <cstddef>
#include <string>

#include <OgreSharedPtr.h>
#include <OgrePrerequisites.h>
#include <OgreBlendMode.h>

#include "nav_msgs/msg/occupancy_grid.hpp"

#include "rviz_default_plugins/visibility_control.hpp"

namespace Ogre
{
class SceneManager;
class SceneNode;
class ManualObject;
}

namespace rviz_default_plugins
{
namespace displays
{
class MapDisplay;

class Swatch
{
public:
  RVIZ_DEFAULT_PLUGINS_PUBLIC
  Swatch(
    Ogre::SceneManager * scene_manager,
    Ogre::SceneNode * parent_scene_node,
    size_t x, size_t y, size_t width, size_t height,
    float resolution, bool draw_under);

  RVIZ_DEFAULT_PLUGINS_PUBLIC
  ~Swatch();

  RVIZ_DEFAULT_PLUGINS_PUBLIC
  void updateAlpha(
    const Ogre::SceneBlendType & sceneBlending, bool depth_write, float alpha);

  RVIZ_DEFAULT_PLUGINS_PUBLIC
  void updateData(const nav_msgs::msg::OccupancyGrid & map);

  RVIZ_DEFAULT_PLUGINS_PUBLIC
  void setVisible(bool visible);

  RVIZ_DEFAULT_PLUGINS_PUBLIC
  void resetOldTexture();

  RVIZ_DEFAULT_PLUGINS_PUBLIC
  void setRenderQueueGroup(uint8_t group);

  RVIZ_DEFAULT_PLUGINS_PUBLIC
  void setDepthWriteEnabled(bool depth_write_enabled);

  RVIZ_DEFAULT_PLUGINS_PUBLIC
  Ogre::Pass * getTechniquePass();

  RVIZ_DEFAULT_PLUGINS_PUBLIC
  std::string getTextureName();

private:
  void setupMaterial();
  void resetTexture(Ogre::DataStreamPtr & pixel_stream);
  void setupSceneNodeWithManualObject();
  void setupSquareManualObject();
  void addPointWithPlaneCoordinates(float x, float y);

  static size_t material_count_;
  static size_t map_count_;
  static size_t node_count_;
  static size_t texture_count_;

  Ogre::SceneManager * scene_manager_;
  Ogre::SceneNode * parent_scene_node_;
  Ogre::SceneNode * scene_node_;
  Ogre::ManualObject * manual_object_;
  Ogre::TexturePtr texture_;
  Ogre::TexturePtr old_texture_;
  Ogre::MaterialPtr material_;
  size_t x_, y_, width_, height_;
};

}  // namespace displays
}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__MAP__SWATCH_HPP_
