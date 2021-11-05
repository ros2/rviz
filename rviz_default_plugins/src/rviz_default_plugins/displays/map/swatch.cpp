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

#include "rviz_default_plugins/displays/map/swatch.hpp"

#include <algorithm>
#include <string>
#include <vector>

#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreRenderable.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreSharedPtr.h>
#include <OgreTechnique.h>
#include <OgreTextureManager.h>

#include "rviz_rendering/custom_parameter_indices.hpp"

namespace rviz_default_plugins
{
namespace displays
{
template<>
size_t SwatchBase<nav_msgs::msg::OccupancyGrid>::material_count_ = 0;
template<>
size_t SwatchBase<nav_msgs::msg::OccupancyGrid>::map_count_ = 0;
template<>
size_t SwatchBase<nav_msgs::msg::OccupancyGrid>::node_count_ = 0;
template<>
size_t SwatchBase<nav_msgs::msg::OccupancyGrid>::texture_count_ = 0;


Swatch::Swatch(
  Ogre::SceneManager * scene_manager,
  Ogre::SceneNode * parent_scene_node,
  size_t x, size_t y, size_t width, size_t height,
  float resolution, bool draw_under)
: SwatchBase<nav_msgs::msg::OccupancyGrid>(
    scene_manager, parent_scene_node,
    x, y, width, height,
    resolution, draw_under
)
{}

void Swatch::updateData(const nav_msgs::msg::OccupancyGrid & map)
{
  size_t pixels_size = width_ * height_;
  size_t map_size = map.data.size();
  size_t map_width = map.info.width;

  auto pixels = std::vector<unsigned char>(pixels_size, 255);

  auto pixel_data = pixels.begin();
  for (size_t map_row = y_; map_row < y_ + height_; map_row++) {
    size_t pixel_index = map_row * map_width + x_;
    size_t pixels_to_copy = std::min(width_, map_size - pixel_index);

    auto row_start = map.data.begin() + pixel_index;
    std::copy(row_start, row_start + pixels_to_copy, pixel_data);
    pixel_data += pixels_to_copy;
    if (pixel_index + pixels_to_copy >= map_size) {
      break;
    }
  }

  Ogre::DataStreamPtr pixel_stream(new Ogre::MemoryDataStream(pixels.data(), pixels_size));

  resetTexture(pixel_stream);
  resetOldTexture();
}
}  // namespace displays
}  // namespace rviz_default_plugins
