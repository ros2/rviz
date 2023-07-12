/*
 * Copyright (c) 2009, Willow Garage, Inc.
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

#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__DEPTH_CLOUD__DEPTH_CLOUD_DISPLAY_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__DEPTH_CLOUD__DEPTH_CLOUD_DISPLAY_HPP_

#ifndef Q_MOC_RUN
#include <QObject>  // NOLINT: cpplint cannot handle the include order here
#include <Ogre.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/message_filter.h>

#include <memory>
#include <mutex>
#include <set>
#include <string>

#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>

#include <rviz_common/depth_cloud_mld.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/editable_enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/status_property.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/transformation/frame_transformer.hpp>

#include <rviz_default_plugins/displays/pointcloud/point_cloud_common.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#endif

#include "rviz_default_plugins/visibility_control.hpp"

#include <QMap>  // NOLINT: cpplint cannot handle the include order here
#include <QString>  // NOLINT: cpplint cannot handle the include order here

namespace rviz_default_plugins
{

namespace displays
{
/**
 * \class DepthCloudDisplay
 *
 */
class RVIZ_DEFAULT_PLUGINS_PUBLIC DepthCloudDisplay : public rviz_common::Display
{
  Q_OBJECT

public:
  DepthCloudDisplay();
  ~DepthCloudDisplay() override;

  void onInitialize() override;

  // Overrides from Display
  void update(float wall_dt, float ros_dt) override;
  void reset() override;
  void setTopic(const QString & topic, const QString & datatype) override;

  void processDepthMessage(const sensor_msgs::msg::Image::ConstSharedPtr msg);
  void processMessage(
    const sensor_msgs::msg::Image::ConstSharedPtr depth_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr rgb_msg);

protected Q_SLOTS:
  void updateQueueSize();
  /** @brief Fill list of available and working transport options */
  void fillTransportOptionList(rviz_common::properties::EnumProperty * property);

  // Property callbacks
  virtual void updateTopic();
  virtual void updateTopicFilter();
  virtual void updateUseAutoSize();
  virtual void updateAutoSizeFactor();
  virtual void updateUseOcclusionCompensation();
  virtual void updateOcclusionTimeOut();

protected:
  void scanForTransportSubscriberPlugins();

  void caminfoCallback(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);

  // overrides from Display
  void onEnable() override;
  void onDisable() override;

  void fixedFrameChanged() override;

  void subscribe();
  void unsubscribe();

  void clear();

  // thread-safe status updates
  // add status update to global status list
  void updateStatus(
    rviz_common::properties::StatusProperty::Level level,
    const QString & name, const QString & text);

  // use global status list to update rviz plugin status
  void setStatusList();

  uint32_t messages_received_;

  // ROS image subscription & synchronization
  std::unique_ptr<image_transport::ImageTransport> depthmap_it_;
  std::shared_ptr<image_transport::SubscriberFilter> depthmap_sub_;
  std::shared_ptr<tf2_ros::MessageFilter<
      sensor_msgs::msg::Image, rviz_common::transformation::FrameTransformer>> depthmap_tf_filter_;
  std::unique_ptr<image_transport::ImageTransport> rgb_it_;
  std::shared_ptr<image_transport::SubscriberFilter> rgb_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr cam_info_;
  std::mutex cam_info_mutex_;

  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::Image, sensor_msgs::msg::Image> SyncPolicyDepthColor;
  typedef message_filters::Synchronizer<SyncPolicyDepthColor> SynchronizerDepthColor;

  std::shared_ptr<SynchronizerDepthColor> sync_depth_color_;

  // RVIZ properties
  rviz_common::properties::Property * topic_filter_property_;
  rviz_common::properties::IntProperty * queue_size_property_;
  rviz_common::properties::BoolProperty * use_auto_size_property_;
  rviz_common::properties::FloatProperty * auto_size_factor_property_;
  rviz_common::properties::RosFilteredTopicProperty * depth_topic_property_;
  rviz_common::properties::EnumProperty * depth_transport_property_;
  rviz_common::properties::RosFilteredTopicProperty * color_topic_property_;
  rviz_common::properties::EnumProperty * color_transport_property_;
  rviz_common::properties::BoolProperty * use_occlusion_compensation_property_;
  rviz_common::properties::FloatProperty * occlusion_shadow_timeout_property_;

  uint32_t queue_size_;

  std::unique_ptr<rviz_common::MultiLayerDepth> ml_depth_data_;

  Ogre::Quaternion current_orientation_;
  Ogre::Vector3 current_position_;
  float angular_thres_;
  float trans_thres_;

  std::unique_ptr<PointCloudCommon> pointcloud_common_;

  std::set<std::string> transport_plugin_types_;
};
}  // namespace displays
}  // namespace rviz_default_plugins
#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__DEPTH_CLOUD__DEPTH_CLOUD_DISPLAY_HPP_
