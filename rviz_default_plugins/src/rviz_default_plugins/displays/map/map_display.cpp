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

#include "map_display.hpp"

#include <memory>
#include <string>

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# pragma GCC diagnostic ignored "-Wpedantic"
#endif

#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTextureManager.h>
#include <OgreTechnique.h>
#include <OgreSharedPtr.h>

#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "rclcpp/time.hpp"

#include "rviz_rendering/custom_parameter_indices.hpp"
#include "rviz_rendering/objects/grid.hpp"
#include "rviz_common/frame_manager_iface.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/msg_conversions.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/int_property.hpp"
#include "rviz_common/properties/property.hpp"
#include "rviz_common/properties/quaternion_property.hpp"
#include "rviz_common/properties/ros_topic_property.hpp"
#include "rviz_common/properties/vector_property.hpp"
#include "rviz_common/validate_floats.hpp"
#include "rviz_common/display_context.hpp"
#include "./palette_builder.hpp"


namespace rviz_default_plugins
{
namespace displays
{

MapDisplay::MapDisplay()
: loaded_(false), resolution_(0.0f), width_(0), height_(0)
{
  connect(this, SIGNAL(mapUpdated()), this, SLOT(showMap()));

  alpha_property_ = new rviz_common::properties::FloatProperty("Alpha", 0.7f,
      "Amount of transparency to apply to the map.",
      this, SLOT(updateAlpha()));
  alpha_property_->setMin(0);
  alpha_property_->setMax(1);

  color_scheme_property_ = new rviz_common::properties::EnumProperty("Color Scheme", "map",
      "How to color the occupancy values.",
      this, SLOT(updatePalette()));
  // Option values here must correspond to indices in palette_textures_ array in onInitialize()
  // below.
  color_scheme_property_->addOption("map", 0);
  color_scheme_property_->addOption("costmap", 1);
  color_scheme_property_->addOption("raw", 2);

  draw_under_property_ = new Property("Draw Behind", false,
      "Rendering option, controls whether or not the map is always"
      " drawn behind everything else.",
      this, SLOT(updateDrawUnder()));

  resolution_property_ = new rviz_common::properties::FloatProperty("Resolution", 0,
      "Resolution of the map. (not editable)", this);
  resolution_property_->setReadOnly(true);

  width_property_ = new rviz_common::properties::IntProperty("Width", 0,
      "Width of the map, in meters. (not editable)", this);
  width_property_->setReadOnly(true);

  height_property_ = new rviz_common::properties::IntProperty("Height", 0,
      "Height of the map, in meters. (not editable)", this);
  height_property_->setReadOnly(true);

  position_property_ = new rviz_common::properties::VectorProperty("Position", Ogre::Vector3::ZERO,
      "Position of the bottom left corner of the map, in meters. (not editable)",
      this);
  position_property_->setReadOnly(true);

  orientation_property_ = new rviz_common::properties::QuaternionProperty(
    "Orientation", Ogre::Quaternion::IDENTITY, "Orientation of the map. (not editable)", this);
  orientation_property_->setReadOnly(true);

  transform_timestamp_property_ = new rviz_common::properties::BoolProperty("Use Timestamp", false,
      "Use map header timestamp when transforming", this, SLOT(transformMap()));
}

MapDisplay::~MapDisplay()
{
  unsubscribe();
  clear();
  swatches_.clear();
}

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
: MapDisplay()
{
  context_ = context;
  scene_manager_ = context->getSceneManager();
  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  palette_textures_.push_back(makePaletteTexture(makeMapPalette()));
  color_scheme_transparency_.push_back(false);
  palette_textures_.push_back(makePaletteTexture(makeCostmapPalette()));
  color_scheme_transparency_.push_back(true);
  palette_textures_.push_back(makePaletteTexture(makeRawPalette()));
  color_scheme_transparency_.push_back(true);
}

void MapDisplay::onInitialize()
{
  RosTopicDisplay::onInitialize();
  // Order of palette textures here must match option indices for color_scheme_property_ above.
  palette_textures_.push_back(makePaletteTexture(makeMapPalette()));
  color_scheme_transparency_.push_back(false);
  palette_textures_.push_back(makePaletteTexture(makeCostmapPalette()));
  color_scheme_transparency_.push_back(true);
  palette_textures_.push_back(makePaletteTexture(makeRawPalette()));
  color_scheme_transparency_.push_back(true);
}

// TODO(Martin-Idel-SI): Old subscription method to subscribe to two different topics
//   Currently, this display uses the subscription via RosTopicDisplay (only possible for one topic)
//  void MapDisplay::subscribe()
//  {
//    if (!isEnabled()) {
//      return;
//    }
//
//    if (!topic_property_->getTopic().isEmpty()) {
//      try {
//        if (unreliable_property_->getBool()) {
//          map_sub_ = update_nh_.subscribe(topic_property_->getTopicStd(), 1,
//            &MapDisplay::incomingMap, this, ros::TransportHints().unreliable());
//        } else {
//          map_sub_ = update_nh_.subscribe(topic_property_->getTopicStd(), 1,
//            &MapDisplay::incomingMap, this, ros::TransportHints().reliable());
//        }
//        setStatus(StatusProperty::Ok, "Topic", "OK");
//      }
//      catch (ros::Exception & e) {
//        setStatus(StatusProperty::Error, "Topic", QString("Error subscribing: ") + e.what());
//      }
//
//      try {
//        update_sub_ = update_nh_.subscribe(topic_property_->getTopicStd() + "_updates", 1,
//          &MapDisplay::incomingUpdate, this);
//        setStatus(StatusProperty::Ok, "Update Topic", "OK");
//      }
//      catch (ros::Exception & e) {
//        setStatus(StatusProperty::Error, "Update Topic", QString("Error subscribing: ")
//        + e.what());
//      }
//    }
//  }
//
//  void MapDisplay::unsubscribe()
//  {
//    map_sub_.shutdown();
//    update_sub_.shutdown();
//  }

void MapDisplay::updateAlpha()
{
  float alpha = alpha_property_->getFloat();
  Ogre::SceneBlendType sceneBlending;
  bool depthWrite;

  if (alpha < 0.9998 || color_scheme_transparency_[color_scheme_property_->getOptionInt()]) {
    sceneBlending = Ogre::SBT_TRANSPARENT_ALPHA;
    depthWrite = false;
  } else {
    sceneBlending = Ogre::SBT_REPLACE;
    depthWrite = !draw_under_property_->getValue().toBool();
  }

  for (auto swatch : swatches_) {
    swatch->updateAlpha(sceneBlending, depthWrite, alpha);
  }
}

void MapDisplay::updateDrawUnder()
{
  bool draw_under = draw_under_property_->getValue().toBool();

  if (alpha_property_->getFloat() >= 0.9998) {
    for (auto swatch : swatches_) {
      swatch->material_->setDepthWriteEnabled(!draw_under);
    }
  }

  uint8_t group = draw_under ? Ogre::RENDER_QUEUE_4 : Ogre::RENDER_QUEUE_MAIN;
  for (auto swatch : swatches_) {
    if (swatch->manual_object_) {
      swatch->manual_object_->setRenderQueueGroup(group);
    }
  }
}

void MapDisplay::clear()
{
  setStatus(rviz_common::properties::StatusProperty::Warn, "Message", "No map received");

  if (!loaded_) {
    return;
  }

  for (auto swatch : swatches_) {
    if (swatch->manual_object_) {
      swatch->manual_object_->setVisible(false);
    }

    if (!swatch->texture_) {
      Ogre::TextureManager::getSingleton().remove(swatch->texture_);
      swatch->texture_.reset();
    }
  }

  loaded_ = false;
}

bool validateFloats(const nav_msgs::msg::OccupancyGrid & msg)
{
  return rviz_common::validateFloats(msg.info.resolution) &&
         rviz_common::validateFloats(msg.info.origin);
}

void MapDisplay::processMessage(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
  current_map_ = *msg;
  loaded_ = true;
  // updated via signal in case ros spinner is in a different thread
  Q_EMIT mapUpdated();
}

// TODO(Martin-Idel-SI): Port when map_msgs is ported to ROS2
//  void MapDisplay::incomingUpdate(const map_msgs::OccupancyGridUpdate::ConstPtr & update)
//  {
//    // Only update the map if we have gotten a full one first.
//    if (!loaded_) {
//      return;
//    }
//
//    // Reject updates which have any out-of-bounds data.
//    if (update->x < 0 ||
//      update->y < 0 ||
//      current_map_.info.width < update->x + update->width ||
//      current_map_.info.height < update->y + update->height) {
//      setStatus(rviz_common::properties::StatusProperty::Error,
//        "Update", "Update area outside of original map area.");
//      return;
//    }
//
//    // Copy the incoming data into current_map_'s data.
//    for (size_t y = 0; y < update->height; y++) {
//      memcpy(&current_map_.data[(update->y + y) * current_map_.info.width + update->x],
//        &update->data[y * update->width],
//        update->width);
//    }
//    // updated via signal in case ros spinner is in a different thread
//    Q_EMIT mapUpdated();
//  }

void MapDisplay::createSwatches()
{
  size_t width = current_map_.info.width;
  size_t height = current_map_.info.height;
  float resolution = current_map_.info.resolution;

  size_t swatch_width = width;
  size_t swatch_height = height;
  int number_swatches = 1;
  int maximum_number_swatch_splittings = 4;  // 4 seems to work well for this purpose.

  for (int i = 0; i < maximum_number_swatch_splittings; i++) {
    RVIZ_COMMON_LOG_INFO_STREAM("Creating " << number_swatches << " swatches_");
    swatches_.clear();
    try {
      tryCreateSwatches(width, height, resolution, swatch_width, swatch_height, number_swatches);
      return;
    } catch (Ogre::InvalidParametersException &) {
      doubleSwatchNumber(swatch_width, swatch_height, number_swatches);
    } catch (Ogre::RenderingAPIException &) {
      // This exception seems no longer thrown on some systems. May still be relevant for others.
      doubleSwatchNumber(swatch_width, swatch_height, number_swatches);
    }
  }
}

void MapDisplay::doubleSwatchNumber(
  size_t & swatch_width, size_t & swatch_height, int & number_swatches) const
{
  RVIZ_COMMON_LOG_ERROR_STREAM("Failed to create " << number_swatches << " swatches_");
  if (swatch_width > swatch_height) {
    swatch_width /= 2;
  } else {
    swatch_height /= 2;
  }
  number_swatches *= 2;
}

void MapDisplay::tryCreateSwatches(
  size_t width,
  size_t height,
  float resolution,
  size_t swatch_width,
  size_t swatch_height,
  int number_swatches)
{
  size_t x = 0;
  size_t y = 0;
  for (int i = 0; i < number_swatches; i++) {
    size_t effective_width = getEffectiveDimension(width, swatch_width, x);
    size_t effective_height = getEffectiveDimension(height, swatch_height, y);

    swatches_.push_back(std::make_shared<Swatch>(
        scene_manager_,
        scene_node_,
        x, y,
        effective_width,
        effective_height,
        resolution,
        draw_under_property_->getValue().toBool()));

    swatches_[i]->updateData(current_map_);

    x += effective_width;
    if (x >= width) {
      x = 0;
      y += effective_height;
    }
  }
  updateAlpha();
}

size_t MapDisplay::getEffectiveDimension(
  size_t map_dimension, size_t swatch_dimension, size_t position)
{
  // Last swatch is bigger than swatch_dimension for odd numbers.
  // subtracting the swatch_dimension in the LHS handles this case.
  return map_dimension - position - swatch_dimension >= swatch_dimension ?
         swatch_dimension :
         map_dimension - position;
}

void MapDisplay::showMap()
{
  if (current_map_.data.empty()) {
    return;
  }

  if (!validateFloats(current_map_)) {
    setStatus(rviz_common::properties::StatusProperty::Error, "Map",
      "Message contained invalid floating point values (nans or infs)");
    return;
  }

  size_t width = current_map_.info.width;
  size_t height = current_map_.info.height;

  if (width * height == 0) {
    std::string message =
      "Map is zero-sized (" + std::to_string(width) + "x" + std::to_string(height) + ")";
    setStatus(
      rviz_common::properties::StatusProperty::Error, "Map", QString::fromStdString(message));
    return;
  }

  if (width * height != current_map_.data.size()) {
    std::string message =
      "Data size doesn't match width*height: width = " + std::to_string(width) + ", height = " +
      std::to_string(height) + ", data size = " + std::to_string(current_map_.data.size());
    setStatus(
      rviz_common::properties::StatusProperty::Error, "Map", QString::fromStdString(message));
    return;
  }

  setStatus(rviz_common::properties::StatusProperty::Ok, "Message", "Map received");

  RVIZ_COMMON_LOG_DEBUG_STREAM("Received a " << current_map_.info.width << " X " <<
    current_map_.info.height << " map @ " << current_map_.info.resolution << "m/pix\n");

  showValidMap();
}

void MapDisplay::showValidMap()
{
  size_t width = current_map_.info.width;
  size_t height = current_map_.info.height;

  float resolution = current_map_.info.resolution;

  resetSwatchesIfNecessary(width, height, resolution);

  frame_ = current_map_.header.frame_id;
  if (frame_.empty()) {
    frame_ = "/map";
  }

  updateSwatches();

  setStatus(rviz_common::properties::StatusProperty::Ok, "Map", "Map OK");
  updatePalette();

  resolution_property_->setValue(resolution);
  width_property_->setValue(static_cast<unsigned int>(width));
  height_property_->setValue(static_cast<unsigned int>(height));

  position_property_->setVector(rviz_common::pointMsgToOgre(current_map_.info.origin.position));
  orientation_property_->setQuaternion(
    rviz_common::quaternionMsgToOgre(current_map_.info.origin.orientation));

  transformMap();

  context_->queueRender();
}

void MapDisplay::resetSwatchesIfNecessary(size_t width, size_t height, float resolution)
{
  if (width != width_ || height != height_ || resolution_ != resolution) {
    createSwatches();
    width_ = width;
    height_ = height;
    resolution_ = resolution;
  }
}

void MapDisplay::updateSwatches() const
{
  for (auto swatch : swatches_) {
    swatch->updateData(current_map_);

    Ogre::Pass * pass = swatch->material_->getTechnique(0)->getPass(0);
    Ogre::TextureUnitState * tex_unit = nullptr;
    if (pass->getNumTextureUnitStates() > 0) {
      tex_unit = pass->getTextureUnitState(0);
    } else {
      tex_unit = pass->createTextureUnitState();
    }

    tex_unit->setTextureName(swatch->texture_->getName());
    tex_unit->setTextureFiltering(Ogre::TFO_NONE);
    swatch->manual_object_->setVisible(true);
  }
}

void MapDisplay::updatePalette()
{
  int palette_index = color_scheme_property_->getOptionInt();

  for (auto swatch : swatches_) {
    Ogre::Pass * pass = swatch->material_->getTechnique(0)->getPass(0);
    Ogre::TextureUnitState * palette_tex_unit = nullptr;
    if (pass->getNumTextureUnitStates() > 1) {
      palette_tex_unit = pass->getTextureUnitState(1);
    } else {
      palette_tex_unit = pass->createTextureUnitState();
    }
    palette_tex_unit->setTextureName(palette_textures_[palette_index]->getName());
    palette_tex_unit->setTextureFiltering(Ogre::TFO_NONE);
  }

  updateAlpha();
}

void MapDisplay::transformMap()
{
  if (!loaded_) {
    return;
  }

  rclcpp::Time transform_time = context_->getClock()->now();

  if (transform_timestamp_property_->getBool()) {
    transform_time = current_map_.header.stamp;
  }

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->transform(frame_, transform_time, current_map_.info.origin,
    position, orientation) &&
    !context_->getFrameManager()->transform(frame_, context_->getClock()->now(),
    current_map_.info.origin, position, orientation))
  {
    RVIZ_COMMON_LOG_ERROR_STREAM("Error transforming map '" << getName().toStdString() <<
      "' from frame '" << frame_ << "' to '" << fixed_frame_.toStdString() << "'.");

    setStatus(rviz_common::properties::StatusProperty::Error, "Transform",
      "No transform from [" + QString::fromStdString(frame_) + "] to [" + fixed_frame_ + "]");
    scene_node_->setVisible(false);
  } else {
    setStatus(rviz_common::properties::StatusProperty::Ok, "Transform", "Transform OK");
  }

  scene_node_->setPosition(position);
  scene_node_->setOrientation(orientation);
}

void MapDisplay::fixedFrameChanged()
{
  transformMap();
}

void MapDisplay::reset()
{
  RosTopicDisplay::reset();
  clear();
  // TODO(Martin-Idel-SI): We can't force resubscription this way anymore.
  // Force resubscription so that the map will be re-sent
  // updateTopic();
}

void MapDisplay::update(float wall_dt, float ros_dt)
{
  (void) wall_dt;
  (void) ros_dt;

  transformMap();
}

}  // namespace displays
}  // namespace rviz_default_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::MapDisplay, rviz_common::Display)
