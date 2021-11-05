/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

#include "rviz_default_plugins/displays/map/map_display.hpp"

#include <memory>
#include <string>
#include <vector>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTextureManager.h>
#include <OgreTechnique.h>


#include "rviz_rendering/objects/grid.hpp"
#include "rviz_common/properties/property.hpp"
#include "rviz_common/validate_floats.hpp"
#include "rviz_common/display_context.hpp"


namespace rviz_default_plugins
{
namespace displays
{

Ogre::TexturePtr makePaletteTexture(std::vector<unsigned char> palette_bytes)
{
  Ogre::DataStreamPtr palette_stream;
  palette_stream.reset(new Ogre::MemoryDataStream(palette_bytes.data(), 256 * 4));

  static int palette_tex_count = 0;
  std::string tex_name = "MapPaletteTexture" + std::to_string(palette_tex_count++);
  return Ogre::TextureManager::getSingleton().loadRawData(
    tex_name, "rviz_rendering", palette_stream, 256, 1, Ogre::PF_BYTE_RGBA, Ogre::TEX_TYPE_1D, 0);
}

MapDisplay::MapDisplay(rviz_common::DisplayContext * context)
: MapDisplayBase<nav_msgs::msg::OccupancyGrid, map_msgs::msg::OccupancyGridUpdate>(context)
{
  // Order of palette textures here must match option indices for color_scheme_property_ above.
  palette_textures_.push_back(makePaletteTexture(makeMapPalette()));
  color_scheme_transparency_.push_back(false);
  palette_textures_.push_back(makePaletteTexture(makeCostmapPalette()));
  color_scheme_transparency_.push_back(true);
  palette_textures_.push_back(makePaletteTexture(makeRawPalette()));
  color_scheme_transparency_.push_back(true);

  // add property for color_map selection
  color_scheme_property_ = new rviz_common::properties::EnumProperty(
    "Color Scheme", "map",
    "How to color the occupancy values.",
    this, SLOT(updatePalette()));
  // Option values here must correspond to indices in palette_textures_ array above
  color_scheme_property_->addOption("map", 0);
  color_scheme_property_->addOption("costmap", 1);
  color_scheme_property_->addOption("raw", 2);
}

void MapDisplay::showValidMap()
{
  MapDisplayBase::showValidMap();
  updatePalette();
}

bool MapDisplay::validateFloats(const nav_msgs::msg::OccupancyGrid & msg) const
{
  return rviz_common::validateFloats(msg.info.resolution) &&
         rviz_common::validateFloats(msg.info.origin);
}

void MapDisplay::incomingUpdate(const map_msgs::msg::OccupancyGridUpdate::ConstSharedPtr update)
{
  // Only update the map if we have gotten a full one first.
  if (!loaded_) {
    return;
  }

  ++update_messages_received_;
  setStatus(
    rviz_common::properties::StatusProperty::Ok,
    "Topic",
    QString::number(update_messages_received_) + " update messages received");

  if (updateDataOutOfBounds(update)) {
    setStatus(
      rviz_common::properties::StatusProperty::Error,
      "Update", "Update area outside of original map area.");
    return;
  }

  updateMapDataInMemory(update);
  setStatus(rviz_common::properties::StatusProperty::Ok, "Update", "Update OK");

  // updated via signal in case ros spinner is in a different thread
  Q_EMIT q_helper_object_->mapUpdated();
}

bool MapDisplay::updateDataOutOfBounds(
  const map_msgs::msg::OccupancyGridUpdate::ConstSharedPtr update) const
{
  return update->x < 0 ||
         update->y < 0 ||
         current_map_.info.width < update->x + update->width ||
         current_map_.info.height < update->y + update->height;
}

void MapDisplay::updateMapDataInMemory(
  const map_msgs::msg::OccupancyGridUpdate::ConstSharedPtr update)
{
  for (size_t y = 0; y < update->height; y++) {
    std::copy(
      update->data.begin(),
      update->data.begin() + update->width,
      current_map_.data.begin() + (update->y + y) * current_map_.info.width + update->x);
  }
}

void MapDisplay::processMessage(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
  current_map_ = *msg;
  loaded_ = true;
  // updated via signal in case ros spinner is in a different thread
  Q_EMIT q_helper_object_->mapUpdated();
}

void MapDisplay::updatePalette()
{
  int palette_index = color_scheme_property_->getOptionInt();

  for (const auto & swatch : swatches_) {
    Ogre::Pass * pass = swatch->getTechniquePass();
    Ogre::TextureUnitState * palette_tex_unit = nullptr;
    if (pass->getNumTextureUnitStates() > 1) {
      palette_tex_unit = pass->getTextureUnitState(1);
    } else {
      palette_tex_unit = pass->createTextureUnitState();
    }
    palette_tex_unit->setTexture(palette_textures_[palette_index]);
    palette_tex_unit->setTextureFiltering(Ogre::TFO_NONE);
  }

  updateAlpha();
  updateDrawUnder();
}

std::shared_ptr<SwatchBase<nav_msgs::msg::OccupancyGrid>> MapDisplay::createSwatch(
  Ogre::SceneManager * scene_manager,
  Ogre::SceneNode * parent_scene_node,
  size_t x,
  size_t y,
  size_t width,
  size_t height,
  float resolution,
  bool draw_under)
{
  // make shared not possible due to abstract base class
  return std::shared_ptr<SwatchBase<nav_msgs::msg::OccupancyGrid>>(
    new Swatch(
      scene_manager,
      parent_scene_node,
      x, y, width, height, resolution, draw_under
  ));
}

}  // namespace displays
}  // namespace rviz_default_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::MapDisplay, rviz_common::Display)
