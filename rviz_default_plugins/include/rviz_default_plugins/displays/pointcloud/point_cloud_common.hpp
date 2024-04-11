/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__POINTCLOUD__POINT_CLOUD_COMMON_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__POINTCLOUD__POINT_CLOUD_COMMON_HPP_

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <deque>
# include <list>
# include <map>
# include <memory>
# include <queue>
# include <vector>
# include <string>

# include <QObject>  // NOLINT
# include <QList>  // NOLINT

//# include <message_filters/time_sequencer.h>

#include "pluginlib/class_loader.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/time.hpp"

#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "rviz_common/interaction/forwards.hpp"
#include "rviz_common/interaction/selection_manager.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_rendering/objects/point_cloud.hpp"

#include "point_cloud_transformer.hpp"
#include "point_cloud_selection_handler.hpp"

#include "rviz_default_plugins/displays/pointcloud/point_cloud_selection_handler.hpp"
#include "rviz_default_plugins/displays/pointcloud/point_cloud_transformer.hpp"
#include "rviz_default_plugins/displays/pointcloud/point_cloud_transformer_factory.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

#endif

namespace rviz_common
{

class Display;
class DisplayContext;

namespace properties
{

class BoolProperty;
class EnumProperty;
class FloatProperty;

}  // namespace properties

}  // namespace rviz_common

namespace rviz_default_plugins
{

typedef std::shared_ptr<PointCloudSelectionHandler> PointCloudSelectionHandlerPtr;
class PointCloudTransformer;
typedef std::shared_ptr<PointCloudTransformer> PointCloudTransformerPtr;

typedef std::vector<std::string> V_string;

struct RVIZ_DEFAULT_PLUGINS_PUBLIC CloudInfo
{
  CloudInfo();
  ~CloudInfo();

  // clear the point cloud, but keep selection handler around
  void clear();

  void setSelectable(
    bool selectable, float selection_box_size, rviz_common::DisplayContext * context);

  rclcpp::Time receive_time_;

  Ogre::SceneManager * manager_;

  sensor_msgs::msg::PointCloud2::ConstSharedPtr message_;

  Ogre::SceneNode * scene_node_;
  std::shared_ptr<rviz_rendering::PointCloud> cloud_;
  PointCloudSelectionHandlerPtr selection_handler_;

  std::vector<rviz_rendering::PointCloud::Point> transformed_points_;

  Ogre::Quaternion orientation_;
  Ogre::Vector3 position_;
};

/**
 * \class PointCloudCommon
 * \brief Displays a point cloud of type sensor_msgs::PointCloud
 *
 * By default it will assume channel 0 of the cloud is an intensity value, and will color them by intensity.
 * If you set the channel's name to "rgb", it will interpret the channel as an integer rgb value, with r, g and b
 * all being 8 bits.
 */
class RVIZ_DEFAULT_PLUGINS_PUBLIC PointCloudCommon : public QObject
{
  Q_OBJECT

public:
  typedef std::shared_ptr<CloudInfo> CloudInfoPtr;
  typedef std::deque<CloudInfoPtr> D_CloudInfo;
  typedef std::vector<CloudInfoPtr> V_CloudInfo;
  typedef std::list<CloudInfoPtr> L_CloudInfo;

  explicit PointCloudCommon(rviz_common::Display * display);

  void initialize(rviz_common::DisplayContext * context, Ogre::SceneNode * scene_node);

  void reset();
  void update(float wall_dt, float ros_dt);

  void addMessage(sensor_msgs::msg::PointCloud::ConstSharedPtr cloud);
  void addMessage(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud);

  rviz_common::Display * getDisplay() {return display_;}

  void onDisable();

  bool auto_size_;

  rviz_common::properties::BoolProperty * selectable_property_;
  rviz_common::properties::FloatProperty * point_world_size_property_;
  rviz_common::properties::FloatProperty * point_pixel_size_property_;
  rviz_common::properties::FloatProperty * alpha_property_;
  rviz_common::properties::EnumProperty * xyz_transformer_property_;
  rviz_common::properties::EnumProperty * color_transformer_property_;
  rviz_common::properties::EnumProperty * style_property_;
  rviz_common::properties::FloatProperty * decay_time_property_;

  void setAutoSize(bool auto_size);

public Q_SLOTS:
  void causeRetransform();

private Q_SLOTS:
  void updateSelectable();
  void updateStyle();
  void updateBillboardSize();
  void updateAlpha();
  void updateXyzTransformer();
  void updateColorTransformer();
  void setXyzTransformerOptions(rviz_common::properties::EnumProperty * prop);
  void setColorTransformerOptions(rviz_common::properties::EnumProperty * prop);

protected:
  void processMessage(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud);

private:
  bool transformCloud(const CloudInfoPtr & cloud, bool fully_update_transformers);
  bool transformPoints(
    const CloudInfoPtr & cloud_info, V_PointCloudPoint & cloud_points, bool update_transformers);
  void setProblematicPointsToInfinity(V_PointCloudPoint & cloud_points);
  void updateStatus();

  PointCloudTransformerPtr getXYZTransformer(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud);
  PointCloudTransformerPtr getColorTransformer(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud);
  void updateTransformers(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud);
  void retransform();

  void loadTransformers();
  void loadTransformer(
    PointCloudTransformerPtr trans,
    std::string name,
    const std::string & lookup_name);

  float getSelectionBoxSize();
  void setPropertiesHidden(const QList<rviz_common::properties::Property *> & props, bool hide);
  void fillTransformerOptions(rviz_common::properties::EnumProperty * prop, uint32_t mask);

  void insertNewClouds(float point_decay_time, const rclcpp::Time & now);
  float getSizeForRenderMode(const rviz_rendering::PointCloud::RenderMode & mode);

  void updateTransformerProperties();

  /**
   * Instead of deleting obsolete cloud infos, we just clear them
   * and put them into obsolete_cloud_infos, so active selections are preserved
   *
   * If decay time == 0, clear the old cloud when we get a new one.
   * Otherwise, clear all the outdated ones
   */
  void collectObsoleteCloudInfos(float point_decay_time, const rclcpp::Time & now);

  /// Garbage-collect old point clouds that don't have an active selection
  void removeObsoleteCloudInfos();

  bool cloudInfoIsDecayed(
    CloudInfoPtr cloud_info, float point_decay_time, const rclcpp::Time & now);

  D_CloudInfo cloud_infos_;

  Ogre::SceneNode * scene_node_;

  V_CloudInfo new_cloud_infos_;
  std::mutex new_clouds_mutex_;

  L_CloudInfo obsolete_cloud_infos_;

  struct TransformerInfo
  {
    PointCloudTransformerPtr transformer;
    QList<rviz_common::properties::Property *> xyz_props;
    QList<rviz_common::properties::Property *> color_props;

    std::string readable_name;
    std::string lookup_name;
  };
  typedef std::map<std::string, TransformerInfo> M_TransformerInfo;

  std::recursive_mutex transformers_mutex_;
  M_TransformerInfo transformers_;
  bool new_xyz_transformer_;
  bool new_color_transformer_;
  bool needs_retransform_;

  std::unique_ptr<PointCloudTransformerFactory> transformer_factory_;

  rviz_common::Display * display_;
  rviz_common::DisplayContext * context_;
  rclcpp::Clock::SharedPtr clock_;

  static const std::string message_status_name_;

  friend class PointCloudSelectionHandler;
};

}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__POINTCLOUD__POINT_CLOUD_COMMON_HPP_
