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
#include <string>
#include <vector>

#ifndef Q_MOC_RUN

#include <OgreTexture.h>
#include <OgreMaterial.h>
#include <OgreVector.h>
#include <OgreSharedPtr.h>

#endif  // Q_MOC_RUN

#include "nav_msgs/msg/map_meta_data.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "map_msgs/msg/occupancy_grid_update.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/qos.hpp"

#include "rviz_common/message_filter_display.hpp"

#include "rviz_default_plugins/displays/map/swatch.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

namespace Ogre
{
class ManualObject;
}

namespace rviz_common
{
namespace properties
{

class EnumProperty;
class FloatProperty;
class IntProperty;
class Property;
class QuaternionProperty;
class VectorProperty;

}  // namespace properties
}  // namespace rviz_common

namespace rviz_default_plugins
{
namespace displays
{
class AlphaSetter;


/**
 * \class MapDisplay
 * \brief Displays a map along the XY plane.
 */
class RVIZ_DEFAULT_PLUGINS_PUBLIC MapDisplay : public
  rviz_common::MessageFilterDisplay<nav_msgs::msg::OccupancyGrid>
{
  Q_OBJECT

public:
  // TODO(botteroa-si): Constructor for testing, remove once ros_nodes can be mocked and call
  // initialize() instead
  explicit MapDisplay(rviz_common::DisplayContext * context);
  MapDisplay();
  ~MapDisplay() override;

  void onInitialize() override;
  void fixedFrameChanged() override;
  void reset() override;

  float getResolution() {return resolution_;}
  size_t getWidth() {return width_;}
  size_t getHeight() {return height_;}

  /** @brief Copy msg into current_map_ and call showMap(). */
  void processMessage(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) override;

public Q_SLOTS:
  void showMap();

Q_SIGNALS:
  /** @brief Emitted when a new map is received*/
  void mapUpdated();

protected Q_SLOTS:
  void updateAlpha();
  void updateDrawUnder() const;
  void updatePalette();
  void updateBinaryThreshold();
  /** @brief Show current_map_ in the scene. */
  void transformMap();
  void updateMapUpdateTopic();

protected:
  void updateTopic() override;
  void update(float wall_dt, float ros_dt) override;

  void subscribe() override;
  void unsubscribe() override;

  void onEnable() override;

  /** @brief Copy update's data into current_map_ and call showMap(). */
  void incomingUpdate(map_msgs::msg::OccupancyGridUpdate::ConstSharedPtr update);

  bool updateDataOutOfBounds(map_msgs::msg::OccupancyGridUpdate::ConstSharedPtr update) const;
  void updateMapDataInMemory(map_msgs::msg::OccupancyGridUpdate::ConstSharedPtr update);

  void clear();

  void subscribeToUpdateTopic();
  void unsubscribeToUpdateTopic();

  void showValidMap();
  void resetSwatchesIfNecessary(size_t width, size_t height, float resolution);
  void createSwatches();
  void doubleSwatchNumber(
    size_t & swatch_width, size_t & swatch_height,
    int & number_swatches) const;
  void tryCreateSwatches(
    size_t width,
    size_t height,
    float resolution,
    size_t swatch_width,
    size_t swatch_height,
    int number_swatches);
  size_t getEffectiveDimension(size_t map_dimension, size_t swatch_dimension, size_t position);
  void updateSwatches() const;

  std::vector<std::shared_ptr<Swatch>> swatches_;
  std::vector<Ogre::TexturePtr> palette_textures_, palette_textures_binary_;
  std::vector<bool> color_scheme_transparency_;
  bool loaded_;

  float resolution_;
  size_t width_;
  size_t height_;
  std::string frame_;
  nav_msgs::msg::OccupancyGrid current_map_;

  rclcpp::Subscription<map_msgs::msg::OccupancyGridUpdate>::SharedPtr update_subscription_;
  rclcpp::QoS update_profile_;
  rclcpp::Time subscription_start_time_;

  rviz_common::properties::RosTopicProperty * update_topic_property_;
  rviz_common::properties::QosProfileProperty * update_profile_property_;
  rviz_common::properties::FloatProperty * resolution_property_;
  rviz_common::properties::IntProperty * width_property_;
  rviz_common::properties::IntProperty * height_property_;
  rviz_common::properties::VectorProperty * position_property_;
  rviz_common::properties::QuaternionProperty * orientation_property_;
  rviz_common::properties::FloatProperty * alpha_property_;
  rviz_common::properties::Property * draw_under_property_;
  rviz_common::properties::EnumProperty * color_scheme_property_;
  rviz_common::properties::BoolProperty * transform_timestamp_property_;
  rviz_common::properties::BoolProperty * binary_view_property_;
  rviz_common::properties::IntProperty * binary_threshold_property_;

  uint32_t update_messages_received_;
};

}  // namespace displays
}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__MAP__MAP_DISPLAY_HPP_
