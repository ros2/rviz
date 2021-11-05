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
#include "rviz_default_plugins/displays/map/swatch_base.hpp"

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

class Swatch : public SwatchBase<nav_msgs::msg::OccupancyGrid>
{
public:
  RVIZ_DEFAULT_PLUGINS_PUBLIC
  Swatch(
    Ogre::SceneManager * scene_manager,
    Ogre::SceneNode * parent_scene_node,
    size_t x, size_t y, size_t width, size_t height,
    float resolution, bool draw_under);

  RVIZ_DEFAULT_PLUGINS_PUBLIC
  ~Swatch() override = default;

  RVIZ_DEFAULT_PLUGINS_PUBLIC
  void updateData(const nav_msgs::msg::OccupancyGrid & map) override;
};

}  // namespace displays
}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__MAP__SWATCH_HPP_
