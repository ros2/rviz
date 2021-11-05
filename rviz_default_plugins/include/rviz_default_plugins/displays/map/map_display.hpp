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

#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__MAP__MAP_DISPLAY_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__MAP__MAP_DISPLAY_HPP_

#include <memory>
#include <vector>

#include "nav_msgs/msg/map_meta_data.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "map_msgs/msg/occupancy_grid_update.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/qos.hpp"

#include "rviz_common/properties/enum_property.hpp"
#include "rviz_default_plugins/displays/map/map_display_base.hpp"
#include "rviz_default_plugins/displays/map/swatch.hpp"
#include "rviz_default_plugins/visibility_control.hpp"


namespace rviz_default_plugins
{
namespace displays
{

/**
 * \class MapDisplay
 * \brief Displays a map along the XY plane.
 */
class RVIZ_DEFAULT_PLUGINS_PUBLIC MapDisplay : public
  MapDisplayBase<nav_msgs::msg::OccupancyGrid, map_msgs::msg::OccupancyGridUpdate>
{
  Q_OBJECT

public:
  // TODO(botteroa-si): Constructor for testing, remove once ros_nodes can be mocked and call
  // initialize() instead
  explicit MapDisplay(rviz_common::DisplayContext * context);
  MapDisplay() = default;
  ~MapDisplay() override = default;

  /** @brief Copy msg into current_map_ and call showMap(). */
  void processMessage(MsgConstSharedPtr msg) override;

  protected Q_SLOT:
  void updatePalette();

protected:
  void showValidMap() override;

  std::shared_ptr<SwatchBase<nav_msgs::msg::OccupancyGrid>> createSwatch(
    Ogre::SceneManager * scene_manager,
    Ogre::SceneNode * parent_scene_node,
    size_t x, size_t y, size_t width, size_t height,
    float resolution, bool draw_under
  ) override;

  bool validateFloats(const nav_msgs::msg::OccupancyGrid & msg) const override;

  /** @brief Copy update's data into current_map_ and call showMap(). */
  void incomingUpdate(UpdateMsgConstSharedPtr update);

  bool updateDataOutOfBounds(UpdateMsgConstSharedPtr update) const override;
  void updateMapDataInMemory(UpdateMsgConstSharedPtr update) override;

  rviz_common::properties::EnumProperty * color_scheme_property_;

  std::vector<Ogre::TexturePtr> palette_textures_;
  std::vector<bool> color_scheme_transparency_;
};

}  // namespace displays
}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__MAP__MAP_DISPLAY_HPP_
