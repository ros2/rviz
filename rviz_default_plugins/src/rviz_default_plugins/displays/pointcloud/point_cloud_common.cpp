/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

#include "point_cloud_common.hpp"

#include <memory>
#include <set>
#include <string>
#include <utility>

#include <OgreSceneNode.h>
#include <OgreWireBoundingBox.h>

// TODO(wjwwood): revisit file when pluginlib is available
// #include <pluginlib/class_loader.h>
#include "rclcpp/clock.hpp"

#include "./point_cloud_to_point_cloud2.hpp"
#include "rviz_common/display.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/frame_manager.hpp"
#include "rviz_rendering/point_cloud.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/vector_property.hpp"
#include "rviz_common/uniform_string_stream.hpp"
#include "rviz_common/validate_floats.hpp"

#include "src/rviz_default_plugins/displays/pointcloud/transformers/rgb8_pc_transformer.hpp"
#include "src/rviz_default_plugins/displays/pointcloud/transformers/intensity_pc_transformer.hpp"
#include "src/rviz_default_plugins/displays/pointcloud/transformers/xyz_pc_transformer.hpp"
#include "src/rviz_default_plugins/displays/pointcloud/transformers/rgbf32_pc_transformer.hpp"
#include "src/rviz_default_plugins/displays/pointcloud/transformers/axis_color_pc_transformer.hpp"
#include "src/rviz_default_plugins/displays/pointcloud/transformers/flat_color_pc_transformer.hpp"

namespace rviz_default_plugins
{

CloudInfo::CloudInfo()
: manager_(nullptr),
  scene_node_(nullptr),
  position_(Ogre::Vector3::ZERO)
{}

CloudInfo::~CloudInfo()
{
  clear();
}

void CloudInfo::clear()
{
  if (scene_node_) {
    manager_->destroySceneNode(scene_node_);
    scene_node_ = nullptr;
  }
}

const std::string PointCloudCommon::message_status_name_ = "Message";  // NOLINT allow std::string

PointCloudCommon::PointCloudCommon(rviz_common::Display * display)
: auto_size_(false),
  new_xyz_transformer_(false),
  new_color_transformer_(false),
  needs_retransform_(false),
  display_(display)
{
  selectable_property_ = new rviz_common::properties::BoolProperty("Selectable", true,
      "Whether or not the points in this point cloud are selectable.",
      display_, SLOT(updateSelectable()), this);

  style_property_ = new rviz_common::properties::EnumProperty("Style", "Flat Squares",
      "Rendering mode to use, in order of computational complexity.",
      display_, SLOT(updateStyle()), this);
  style_property_->addOption("Points", rviz_rendering::PointCloud::RM_POINTS);
  style_property_->addOption("Squares", rviz_rendering::PointCloud::RM_SQUARES);
  style_property_->addOption("Flat Squares", rviz_rendering::PointCloud::RM_FLAT_SQUARES);
  style_property_->addOption("Spheres", rviz_rendering::PointCloud::RM_SPHERES);
  style_property_->addOption("Boxes", rviz_rendering::PointCloud::RM_BOXES);
  style_property_->addOption("Tiles", rviz_rendering::PointCloud::RM_TILES);

  point_world_size_property_ = new rviz_common::properties::FloatProperty("Size (m)", 0.01,
      "Point size in meters.",
      display_, SLOT(updateBillboardSize()), this);
  point_world_size_property_->setMin(0.0001);

  point_pixel_size_property_ = new rviz_common::properties::FloatProperty("Size (Pixels)", 3,
      "Point size in pixels.",
      display_, SLOT(updateBillboardSize()), this);
  point_pixel_size_property_->setMin(1);

  alpha_property_ = new rviz_common::properties::FloatProperty("Alpha", 1.0,
      "Amount of transparency to apply to the points.  Note that this is experimental "
      "and does not always look correct.",
      display_, SLOT(updateAlpha()), this);
  alpha_property_->setMin(0);
  alpha_property_->setMax(1);

  decay_time_property_ = new rviz_common::properties::FloatProperty("Decay Time", 0,
      "Duration, in seconds, to keep the incoming points.  0 means only show the latest points.",
      display_, SLOT(queueRender()));
  decay_time_property_->setMin(0);

  xyz_transformer_property_ = new rviz_common::properties::EnumProperty("Position Transformer", "",
      "Set the transformer to use to set the position of the points.",
      display_, SLOT(updateXyzTransformer()), this);
  connect(xyz_transformer_property_, SIGNAL(requestOptions(
      rviz_common::properties::EnumProperty *)),
    this, SLOT(setXyzTransformerOptions(rviz_common::properties::EnumProperty *)));

  color_transformer_property_ = new rviz_common::properties::EnumProperty("Color Transformer", "",
      "Set the transformer to use to set the color of the points.",
      display_, SLOT(updateColorTransformer()), this);
  connect(color_transformer_property_,
    SIGNAL(requestOptions(rviz_common::properties::EnumProperty *)),
    this, SLOT(setColorTransformerOptions(rviz_common::properties::EnumProperty *)));
}

void PointCloudCommon::initialize(
  rviz_common::DisplayContext * context,
  Ogre::SceneNode * scene_node)
{
  // TODO(Martin-Idel-SI): revisit when pluginlib is available
//  transformer_class_loader_ = new pluginlib::ClassLoader<PointCloudTransformer>( "rviz",
// "rviz::PointCloudTransformer" );
  loadTransformers();

  context_ = context;
  scene_node_ = scene_node;
  clock_ = context->getClock();

  updateStyle();
  updateBillboardSize();
  updateAlpha();
  updateSelectable();
}

PointCloudCommon::~PointCloudCommon()
{
  // TODO(Martin-Idel-SI): revisit when pluginlib is available
//  if ( transformer_class_loader_ )
//  {
//    delete transformer_class_loader_;
//  }
}

void PointCloudCommon::loadTransformers()
{
  loadTransformer(
    std::make_shared<AxisColorPCTransformer>(), "AxisColorPCTransformer",
    "AxisColorPCTransformer");
  loadTransformer(
    std::make_shared<FlatColorPCTransformer>(), "FlatColorPCTransformer",
    "FlatColorPCTransformer");
  loadTransformer(
    std::make_shared<IntensityPCTransformer>(), "IntensityPCTransformer",
    "IntensityPCTransformer");
  loadTransformer(
    std::make_shared<RGB8PCTransformer>(), "RGB8PCTransformer",
    "RGB8PCTransformer");
  loadTransformer(
    std::make_shared<RGBF32PCTransformer>(), "RGBF32PCTransformer",
    "RGBF32PCTransformer");
  loadTransformer(
    std::make_shared<XYZPCTransformer>(), "XYZPCTransformer",
    "XYZPCTransformer");

// TODO(Martin-Idel-SI): revisit when pluginlib is available
//  std::vector<std::string> classes = transformer_class_loader_->getDeclaredClasses();
//  std::vector<std::string>::iterator ci;
//
//  for( ci = classes.begin(); ci != classes.end(); ci++ )
//  {
//    const std::string& lookup_name = *ci;
//    std::string name = transformer_class_loader_->getName( lookup_name );
//
//    if( transformers_.count( name ) > 0 )
//    {
//      ROS_ERROR( "Transformer type [%s] is already loaded.", name.c_str() );
//      continue;
//    }
//
//    PointCloudTransformerPtr trans( transformer_class_loader_
// ->createUnmanagedInstance( lookup_name ));
//    trans->init();
//    connect( trans.get(), SIGNAL( needRetransform() ), this, SLOT( causeRetransform() ));
//
//    TransformerInfo info;
//    info.transformer = trans;
//    info.readable_name = name;
//    info.lookup_name = lookup_name;
//
//    info.transformer->createProperties( display_, PointCloudTransformer::Support_XYZ,
// info.xyz_props );
//    setPropertiesHidden( info.xyz_props, true );
//
//    info.transformer->createProperties( display_, PointCloudTransformer::Support_Color,
// info.color_props );
//    setPropertiesHidden( info.color_props, true );
//
//    transformers_[ name ] = info;
//  }
}

void PointCloudCommon::loadTransformer(
  PointCloudTransformerPtr trans,
  std::string name,
  const std::string & lookup_name)
{
  trans->init();
  connect(trans.get(), SIGNAL(needRetransform()), this, SLOT(causeRetransform()));

  TransformerInfo info;
  info.transformer = trans;
  info.readable_name = name;
  info.lookup_name = lookup_name;

  info.transformer->createProperties(display_, PointCloudTransformer::Support_XYZ, info.xyz_props);
  setPropertiesHidden(info.xyz_props, true);

  info.transformer->createProperties(display_, PointCloudTransformer::Support_Color,
    info.color_props);
  setPropertiesHidden(info.color_props, true);

  transformers_[name] = info;
}

void PointCloudCommon::setAutoSize(bool auto_size)
{
  auto_size_ = auto_size;
  for (auto const & cloud_info : cloud_infos_) {
    cloud_info->cloud_->setAutoSize(auto_size);
  }
}

void PointCloudCommon::updateAlpha()
{
  for (auto const & cloud_info : cloud_infos_) {
    bool per_point_alpha = findChannelIndex(cloud_info->message_, "rgba") != -1;
    cloud_info->cloud_->setAlpha(alpha_property_->getFloat(), per_point_alpha);
  }
}

void PointCloudCommon::updateSelectable()
{
  bool selectable = selectable_property_->getBool();

  if (selectable) {
    for (auto const & cloud_info : cloud_infos_) {
      cloud_info->selection_handler_.reset(
        new PointCloudSelectionHandler(getSelectionBoxSize(), cloud_info.get(), context_));
      cloud_info->cloud_->setPickColor(
        rviz_common::selection::SelectionManager::handleToColor(
          cloud_info->selection_handler_->getHandle()));
    }
  } else {
    for (auto const & cloud_info : cloud_infos_) {
      cloud_info->selection_handler_.reset();
      cloud_info->cloud_->setPickColor(Ogre::ColourValue(0.0f, 0.0f, 0.0f, 0.0f));
    }
  }
}

void PointCloudCommon::updateStyle()
{
  auto mode = static_cast<rviz_rendering::PointCloud::RenderMode>(style_property_->getOptionInt());
  if (mode == rviz_rendering::PointCloud::RM_POINTS) {
    point_world_size_property_->hide();
    point_pixel_size_property_->show();
  } else {
    point_world_size_property_->show();
    point_pixel_size_property_->hide();
  }
  for (auto const & cloud_info : cloud_infos_) {
    cloud_info->cloud_->setRenderMode(mode);
  }
  updateBillboardSize();
}

void PointCloudCommon::updateBillboardSize()
{
  auto mode = static_cast<rviz_rendering::PointCloud::RenderMode>(style_property_->getOptionInt());
  float size;
  if (mode == rviz_rendering::PointCloud::RM_POINTS) {
    size = point_pixel_size_property_->getFloat();
  } else {
    size = point_world_size_property_->getFloat();
  }
  for (auto & cloud_info : cloud_infos_) {
    cloud_info->cloud_->setDimensions(size, size, size);
    cloud_info->selection_handler_->setBoxSize(getSelectionBoxSize());
  }
  context_->queueRender();
}

void PointCloudCommon::reset()
{
  std::unique_lock<std::mutex> lock(new_clouds_mutex_);
  cloud_infos_.clear();
  new_cloud_infos_.clear();
}

void PointCloudCommon::causeRetransform()
{
  needs_retransform_ = true;
}

void PointCloudCommon::update(float wall_dt, float ros_dt)
{
  (void) wall_dt;
  (void) ros_dt;
  auto mode = static_cast<rviz_rendering::PointCloud::RenderMode>(style_property_->getOptionInt());

  float point_decay_time = decay_time_property_->getFloat();
  if (needs_retransform_) {
    retransform();
    needs_retransform_ = false;
  }

  // instead of deleting cloud infos, we just clear them
  // and put them into obsolete_cloud_infos, so active selections
  // are preserved

  rclcpp::Time now = clock_->now();

  // if decay time == 0, clear the old cloud when we get a new one
  // otherwise, clear all the outdated ones
  {
    std::unique_lock<std::mutex> lock(new_clouds_mutex_);
    if (point_decay_time > 0.0 || !new_cloud_infos_.empty()) {
      while (!cloud_infos_.empty() &&
        (now.nanoseconds() - cloud_infos_.front()->receive_time_.nanoseconds()) / 1000000000.0 >
        point_decay_time)
      {
        cloud_infos_.front()->clear();
        obsolete_cloud_infos_.push_back(cloud_infos_.front());
        cloud_infos_.pop_front();
        context_->queueRender();
      }
    }
  }

  // garbage-collect old point clouds that don't have an active selection
  L_CloudInfo::iterator it = obsolete_cloud_infos_.begin();
  L_CloudInfo::iterator end = obsolete_cloud_infos_.end();
  while (it != end) {
    if (!(*it)->selection_handler_.get() ||
      !(*it)->selection_handler_->hasSelections())
    {
      it = obsolete_cloud_infos_.erase(it);
    }
    if (it != end) {
      ++it;
    }
  }

  {
    std::unique_lock<std::mutex> lock(new_clouds_mutex_);
    if (!new_cloud_infos_.empty()) {
      float size;
      if (mode == rviz_rendering::PointCloud::RM_POINTS) {
        size = point_pixel_size_property_->getFloat();
      } else {
        size = point_world_size_property_->getFloat();
      }

      V_CloudInfo::iterator it = new_cloud_infos_.begin();
      V_CloudInfo::iterator end = new_cloud_infos_.end();
      for (; it != end; ++it) {
        CloudInfoPtr cloud_info = *it;

        V_CloudInfo::iterator next = it; next++;
        // ignore point clouds that are too old, but keep at least one
        if (next != end &&
          (now.nanoseconds() - cloud_info->receive_time_.nanoseconds()) / 1000000000.0 >
          point_decay_time)
        {
          continue;
        }

        bool per_point_alpha = findChannelIndex(cloud_info->message_, "rgba") != -1;

        cloud_info->cloud_.reset(new rviz_rendering::PointCloud());
        cloud_info->cloud_->setRenderMode(mode);
        cloud_info->cloud_->addPoints(
          cloud_info->transformed_points_.begin(), cloud_info->transformed_points_.end());
        cloud_info->cloud_->setAlpha(alpha_property_->getFloat(), per_point_alpha);
        cloud_info->cloud_->setDimensions(size, size, size);
        cloud_info->cloud_->setAutoSize(auto_size_);

        cloud_info->manager_ = context_->getSceneManager();

        cloud_info->scene_node_ = scene_node_->createChildSceneNode(cloud_info->position_,
            cloud_info->orientation_);

        cloud_info->scene_node_->attachObject(cloud_info->cloud_.get());

        cloud_info->selection_handler_.reset(new PointCloudSelectionHandler(getSelectionBoxSize(),
          cloud_info.get(), context_));

        cloud_infos_.push_back(*it);
      }

      new_cloud_infos_.clear();
    }
  }

  {
    std::unique_lock<std::mutex> lock(new_clouds_mutex_);

    if (lock.owns_lock()) {
      if (new_xyz_transformer_ || new_color_transformer_) {
        for (auto transformer : transformers_) {
          const std::string & name = transformer.first;
          TransformerInfo & info = transformer.second;

          setPropertiesHidden(info.xyz_props, name != xyz_transformer_property_->getStdString());
          setPropertiesHidden(info.color_props,
            name != color_transformer_property_->getStdString());
        }
      }
    }

    new_xyz_transformer_ = false;
    new_color_transformer_ = false;
  }

  updateStatus();
}

void PointCloudCommon::setPropertiesHidden(
  const QList<rviz_common::properties::Property *> & props,
  bool hide)
{
  for (auto prop : props) {
    prop->setHidden(hide);
  }
}

void PointCloudCommon::updateTransformers(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud)
{
  std::string xyz_name = xyz_transformer_property_->getStdString();
  std::string color_name = color_transformer_property_->getStdString();

  xyz_transformer_property_->clearOptions();
  color_transformer_property_->clearOptions();

  // Get the channels that we could potentially render
  typedef std::set<std::pair<uint8_t, std::string>> S_string;
  S_string valid_xyz, valid_color;
  bool cur_xyz_valid = false;
  bool cur_color_valid = false;
  bool has_rgb_transformer = false;
  for (auto transformer : transformers_) {
    const std::string & name = transformer.first;
    const PointCloudTransformerPtr & trans = transformer.second.transformer;
    uint32_t mask = trans->supports(cloud);
    if (mask & PointCloudTransformer::Support_XYZ) {
      valid_xyz.insert(std::make_pair(trans->score(cloud), name));
      if (name == xyz_name) {
        cur_xyz_valid = true;
      }
      xyz_transformer_property_->addOptionStd(name);
    }

    if (mask & PointCloudTransformer::Support_Color) {
      valid_color.insert(std::make_pair(trans->score(cloud), name));
      if (name == color_name) {
        cur_color_valid = true;
      }
      if (name == "RGB8") {
        has_rgb_transformer = true;
      }
      color_transformer_property_->addOptionStd(name);
    }
  }

  if (!cur_xyz_valid) {
    if (!valid_xyz.empty()) {
      xyz_transformer_property_->setStringStd(valid_xyz.rbegin()->second);
    }
  }

  if (!cur_color_valid) {
    if (!valid_color.empty()) {
      if (has_rgb_transformer) {
        color_transformer_property_->setStringStd("RGB8");
      } else {
        color_transformer_property_->setStringStd(valid_color.rbegin()->second);
      }
    }
  }
}

void PointCloudCommon::updateStatus()
{
  std::stringstream ss;
  uint32_t total_point_count = 0;
  for (const auto & cloud_info : cloud_infos_) {
    total_point_count += cloud_info->transformed_points_.size();
  }
  ss << "Showing [" << total_point_count << "] points from [" << cloud_infos_.size() <<
    "] messages";
  display_->setStatusStd(rviz_common::properties::StatusProperty::Ok, "Points", ss.str());
}

void PointCloudCommon::processMessage(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
{
  CloudInfoPtr info(new CloudInfo);
  info->message_ = cloud;
  info->receive_time_ = clock_->now();

  if (transformCloud(info, true)) {
    std::unique_lock<std::mutex> lock(new_clouds_mutex_);
    new_cloud_infos_.push_back(info);
    display_->emitTimeSignal(cloud->header.stamp);
  }
}

void PointCloudCommon::updateXyzTransformer()
{
  std::unique_lock<std::recursive_mutex> lock(transformers_mutex_);
  if (transformers_.count(xyz_transformer_property_->getStdString()) == 0) {
    return;
  }
  new_xyz_transformer_ = true;
  causeRetransform();
}

void PointCloudCommon::updateColorTransformer()
{
  std::unique_lock<std::recursive_mutex> lock(transformers_mutex_);
  if (transformers_.count(color_transformer_property_->getStdString()) == 0) {
    return;
  }
  new_color_transformer_ = true;
  causeRetransform();
}

PointCloudTransformerPtr PointCloudCommon::getXYZTransformer(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud)
{
  std::unique_lock<std::recursive_mutex> lock(transformers_mutex_);
  auto it = transformers_.find(xyz_transformer_property_->getStdString());
  if (it != transformers_.end()) {
    const PointCloudTransformerPtr & trans = it->second.transformer;
    if (trans->supports(cloud) & PointCloudTransformer::Support_XYZ) {
      return trans;
    }
  }

  return PointCloudTransformerPtr();
}

PointCloudTransformerPtr PointCloudCommon::getColorTransformer(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud)
{
  std::unique_lock<std::recursive_mutex> lock(transformers_mutex_);
  auto it = transformers_.find(color_transformer_property_->getStdString());
  if (it != transformers_.end()) {
    const PointCloudTransformerPtr & trans = it->second.transformer;
    if (trans->supports(cloud) & PointCloudTransformer::Support_Color) {
      return trans;
    }
  }

  return PointCloudTransformerPtr();
}


void PointCloudCommon::retransform()
{
  std::unique_lock<std::recursive_mutex> lock(transformers_mutex_);

  for (auto const & cloud_info : cloud_infos_) {
    transformCloud(cloud_info, false);
    cloud_info->cloud_->clear();
    cloud_info->cloud_->addPoints(
      cloud_info->transformed_points_.begin(), cloud_info->transformed_points_.end());
  }
}

bool PointCloudCommon::transformCloud(const CloudInfoPtr & cloud_info, bool update_transformers)
{
  if (!cloud_info->scene_node_) {
    if (!context_->getFrameManager()->getTransform(
        cloud_info->message_->header,
        cloud_info->position_,
        cloud_info->orientation_))
    {
      std::stringstream ss;
      ss << "Failed to transform from frame [" << cloud_info->message_->header.frame_id << "] to "
        "frame [" << context_->getFrameManager()->getFixedFrame() << "]";
      display_->setStatusStd(
        rviz_common::properties::StatusProperty::Error, message_status_name_, ss.str());
      return false;
    }
  }
  // Remove outdated error message
  display_->deleteStatusStd(message_status_name_);

  V_PointCloudPoint & cloud_points = cloud_info->transformed_points_;
  cloud_points.clear();

  size_t size = cloud_info->message_->width * cloud_info->message_->height;
  rviz_rendering::PointCloud::Point default_pt = {Ogre::Vector3::ZERO, Ogre::ColourValue(1, 1, 1)};
  cloud_points.resize(size, default_pt);

  if (!transformPoints(cloud_info, cloud_points, update_transformers)) {
    return false;
  }

  setProblematicPointsToInfinity(cloud_points);
  return true;
}

bool PointCloudCommon::transformPoints(
  const CloudInfoPtr & cloud_info, V_PointCloudPoint & cloud_points, bool update_transformers)
{
  Ogre::Matrix4 transform;
  transform.makeTransform(cloud_info->position_, Ogre::Vector3(1, 1, 1), cloud_info->orientation_);

  std::unique_lock<std::recursive_mutex> lock(transformers_mutex_);
  if (update_transformers) {
    updateTransformers(cloud_info->message_);
  }
  PointCloudTransformerPtr xyz_trans = getXYZTransformer(cloud_info->message_);
  PointCloudTransformerPtr color_trans = getColorTransformer(cloud_info->message_);

  if (!xyz_trans) {
    std::stringstream ss;
    ss << "No position transformer available for cloud";
    display_->setStatusStd(
      rviz_common::properties::StatusProperty::Error, message_status_name_, ss.str());
    return false;
  }

  if (!color_trans) {
    std::stringstream ss;
    ss << "No color transformer available for cloud";
    display_->setStatusStd(
      rviz_common::properties::StatusProperty::Error, message_status_name_, ss.str());
    return false;
  }

  xyz_trans->transform(cloud_info->message_, PointCloudTransformer::Support_XYZ, transform,
    cloud_points);
  color_trans->transform(cloud_info->message_, PointCloudTransformer::Support_Color, transform,
    cloud_points);
  return true;
}

void PointCloudCommon::setProblematicPointsToInfinity(V_PointCloudPoint & cloud_points)
{
  for (auto & cloud_point : cloud_points) {
    if (!rviz_common::validateFloats(cloud_point.position)) {
      cloud_point.position.x = 999999.0f;
      cloud_point.position.y = 999999.0f;
      cloud_point.position.z = 999999.0f;
    }
  }
}

void PointCloudCommon::addMessage(const sensor_msgs::msg::PointCloud::ConstSharedPtr cloud)
{
  addMessage(convertPointCloudToPointCloud2(cloud));
}

void PointCloudCommon::addMessage(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
{
  processMessage(cloud);
}

void PointCloudCommon::fixedFrameChanged()
{
  reset();
}

void PointCloudCommon::setXyzTransformerOptions(rviz_common::properties::EnumProperty * prop)
{
  fillTransformerOptions(prop, PointCloudTransformer::Support_XYZ);
}

void PointCloudCommon::setColorTransformerOptions(rviz_common::properties::EnumProperty * prop)
{
  fillTransformerOptions(prop, PointCloudTransformer::Support_Color);
}

void PointCloudCommon::fillTransformerOptions(
  rviz_common::properties::EnumProperty * prop,
  uint32_t mask)
{
  prop->clearOptions();

  if (cloud_infos_.empty()) {
    return;
  }

  std::unique_lock<std::recursive_mutex> lock(transformers_mutex_);

  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg = cloud_infos_.front()->message_;

  for (auto transformer : transformers_) {
    const PointCloudTransformerPtr & trans = transformer.second.transformer;
    if ((trans->supports(msg) & mask) == mask) {
      prop->addOption(QString::fromStdString(transformer.first));
    }
  }
}

float PointCloudCommon::getSelectionBoxSize()
{
  if (style_property_->getOptionInt() != rviz_rendering::PointCloud::RM_POINTS) {
    return point_world_size_property_->getFloat();
  } else {
    return 0.004;
  }
}

}  // namespace rviz_default_plugins
