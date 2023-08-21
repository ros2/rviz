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


#include "rviz_default_plugins/displays/depth_cloud/depth_cloud_display.hpp"

#include <Ogre.h>
#include <tf2_ros/message_filter.h>

#include <QRegExp>

#include <iostream>
#include <functional>
#include <utility>
#include <vector>
#include <string>
#include <memory>
#include <mutex>

#include <image_transport/camera_common.hpp>
#include <image_transport/subscriber_plugin.hpp>

#include <rviz_common/properties/property.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/editable_enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/status_property.hpp>  // NOLINT: cpplint cannot handle the include order here
#include <rviz_common/properties/ros_topic_property.hpp>

#include <rviz_common/transformation/frame_transformer.hpp>

#include <rviz_common/frame_manager_iface.hpp>

#include <rviz_common/display_context.hpp>

#include <rviz_common/depth_cloud_mld.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace rviz_default_plugins
{
namespace displays
{
DepthCloudDisplay::DepthCloudDisplay()
: rviz_common::Display()
  , messages_received_(0)
  , depthmap_sub_()
  , rgb_sub_()
  , cam_info_sub_()
  , queue_size_(5)
  , angular_thres_(0.5f)
  , trans_thres_(0.01f)
{
  ml_depth_data_ = std::make_unique<rviz_common::MultiLayerDepth>();
  // Depth map properties
  QRegExp depth_filter("depth");
  depth_filter.setCaseSensitivity(Qt::CaseInsensitive);

  topic_filter_property_ =
    new rviz_common::properties::Property(
    "Topic Filter", true,
    "List only topics with names that relate to depth and color images", this,
    SLOT(updateTopicFilter()));

  depth_topic_property_ = new rviz_common::properties::RosFilteredTopicProperty(
    "Depth Map Topic", "", "sensor_msgs/msg/Image",
    "sensor_msgs::msg::Image topic to subscribe to.", depth_filter, this, SLOT(updateTopic()));

  depth_transport_property_ = new rviz_common::properties::EnumProperty(
    "Depth Map Transport Hint", "raw", "Preferred method of sending images.",
    this, SLOT(updateTopic()));

  QObject::connect(
    depth_transport_property_,
    SIGNAL(requestOptions(rviz_common::properties::EnumProperty*)),
    this,
    SLOT(fillTransportOptionList(rviz_common::properties::EnumProperty*)));

  depth_transport_property_->setStdString("raw");

  // color image properties
  QRegExp color_filter("color|rgb|bgr|gray|mono");
  color_filter.setCaseSensitivity(Qt::CaseInsensitive);

  color_topic_property_ = new rviz_common::properties::RosFilteredTopicProperty(
    "Color Image Topic", "",
    "sensor_msgs/msg/Image",
    "sensor_msgs::msg::Image topic to subscribe to.", color_filter, this, SLOT(updateTopic()));

  color_transport_property_ = new rviz_common::properties::EnumProperty(
    "Color Transport Hint", "raw", "Preferred method of sending images.", this,
    SLOT(updateTopic()));

  QObject::connect(
    color_transport_property_,
    SIGNAL(requestOptions(rviz_common::properties::EnumProperty*)),
    this,
    SLOT(fillTransportOptionList(rviz_common::properties::EnumProperty*)));

  color_transport_property_->setStdString("raw");

  // Queue size property
  queue_size_property_ =
    new rviz_common::properties::IntProperty(
    "Queue Size", queue_size_,
    "Advanced: set the size of the incoming message queue.  Increasing this "
    "is useful if your incoming TF data is delayed significantly from your"
    " image data, but it can greatly increase memory usage if the messages "
    "are big.",
    this, SLOT(updateQueueSize()));
  queue_size_property_->setMin(1);

  use_auto_size_property_ = new rviz_common::properties::BoolProperty(
    "Auto Size", true,
    "Automatically scale each point based on its depth value and the camera parameters.", this,
    SLOT(updateUseAutoSize()));

  auto_size_factor_property_ =
    new rviz_common::properties::FloatProperty(
    "Auto Size Factor", 1, "Scaling factor to be applied to the auto size.",
    use_auto_size_property_, SLOT(updateAutoSizeFactor()), this);
  auto_size_factor_property_->setMin(0.0001f);

  use_occlusion_compensation_property_ =
    new rviz_common::properties::BoolProperty(
    "Occlusion Compensation", false,
    "Keep points alive after they have been occluded by a closer point. "
    "Points are  removed after a timeout or when the camera frame moves.",
    this, SLOT(updateUseOcclusionCompensation()));

  occlusion_shadow_timeout_property_ = new rviz_common::properties::FloatProperty(
    "Occlusion Time-Out", 30.0f,
    "Amount of seconds before removing occluded points from the depth cloud",
    use_occlusion_compensation_property_, SLOT(updateOcclusionTimeOut()), this);
}

void DepthCloudDisplay::onInitialize()
{
  auto rviz_ros_node_ = context_->getRosNodeAbstraction().lock();

  depthmap_it_ = std::make_unique<image_transport::ImageTransport>(
    rviz_ros_node_->get_raw_node());
  rgb_it_ = std::make_unique<image_transport::ImageTransport>(
    rviz_ros_node_->get_raw_node());

  // Instantiate PointCloudCommon class for displaying point clouds
  pointcloud_common_ = std::make_unique<PointCloudCommon>(this);

  updateUseAutoSize();
  updateUseOcclusionCompensation();

  // Scan for available transport plugins
  scanForTransportSubscriberPlugins();

  pointcloud_common_->initialize(context_, scene_node_);
  pointcloud_common_->xyz_transformer_property_->hide();

  depth_topic_property_->initialize(rviz_ros_node_);
  color_topic_property_->initialize(rviz_ros_node_);
}

DepthCloudDisplay::~DepthCloudDisplay()
{
  if (initialized()) {
    unsubscribe();
    pointcloud_common_.reset();
  }
}

void DepthCloudDisplay::setTopic(const QString & topic, const QString & datatype)
{
  if (datatype == "sensor_msgs::msgs::Image") {
    depth_transport_property_->setStdString("raw");
    depth_topic_property_->setString(topic);
  } else {
    int index = topic.lastIndexOf("/");
    if (index == -1) {
      return;
    }
    QString transport = topic.mid(index + 1);
    QString base_topic = topic.mid(0, index);

    depth_transport_property_->setString(transport);
    depth_topic_property_->setString(base_topic);
  }
}

void DepthCloudDisplay::updateQueueSize()
{
  queue_size_ = queue_size_property_->getInt();
}

void DepthCloudDisplay::updateUseAutoSize()
{
  bool use_auto_size = use_auto_size_property_->getBool();
  pointcloud_common_->point_world_size_property_->setReadOnly(use_auto_size);
  pointcloud_common_->setAutoSize(use_auto_size);
  auto_size_factor_property_->setHidden(!use_auto_size);
  if (use_auto_size) {
    use_auto_size_property_->expand();
  }
}

void DepthCloudDisplay::updateAutoSizeFactor()
{
}

void DepthCloudDisplay::updateTopicFilter()
{
  bool enabled = topic_filter_property_->getValue().toBool();
  depth_topic_property_->enableFilter(enabled);
  color_topic_property_->enableFilter(enabled);
}

void DepthCloudDisplay::updateUseOcclusionCompensation()
{
  bool use_occlusion_compensation = use_occlusion_compensation_property_->getBool();
  occlusion_shadow_timeout_property_->setHidden(!use_occlusion_compensation);

  if (use_occlusion_compensation) {
    updateOcclusionTimeOut();
    ml_depth_data_->enableOcclusionCompensation(true);
    use_occlusion_compensation_property_->expand();
  } else {
    ml_depth_data_->enableOcclusionCompensation(false);
  }
}

void DepthCloudDisplay::updateOcclusionTimeOut()
{
  float occlusion_timeout = occlusion_shadow_timeout_property_->getFloat();
  ml_depth_data_->setShadowTimeOut(occlusion_timeout);
}

void DepthCloudDisplay::onEnable()
{
  subscribe();
}

void DepthCloudDisplay::onDisable()
{
  unsubscribe();

  ml_depth_data_->reset();

  clear();
}

void DepthCloudDisplay::subscribe()
{
  if (!isEnabled()) {
    return;
  }

  try {
    // reset all message filters
    sync_depth_color_ = std::make_shared<SynchronizerDepthColor>(
      SyncPolicyDepthColor(queue_size_));
    depthmap_tf_filter_.reset();
    depthmap_sub_ = std::make_shared<image_transport::SubscriberFilter>();
    rgb_sub_ = std::make_shared<image_transport::SubscriberFilter>();
    cam_info_sub_.reset();

    std::string depthmap_topic = depth_topic_property_->getTopicStd();
    std::string color_topic = color_topic_property_->getTopicStd();

    std::string depthmap_transport = depth_transport_property_->getStdString();
    std::string color_transport = color_transport_property_->getStdString();

    auto rviz_ros_node_ = context_->getRosNodeAbstraction().lock();

    if (!depthmap_topic.empty() && !depthmap_transport.empty()) {
      // subscribe to depth map topic
      depthmap_sub_->subscribe(
        rviz_ros_node_->get_raw_node().get(),
        depthmap_topic,
        depthmap_transport);

      depthmap_tf_filter_ =
        std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::Image,
          rviz_common::transformation::FrameTransformer>>(
        *context_->getFrameManager()->getTransformer(),
        fixed_frame_.toStdString(),
        10,
        rviz_ros_node_->get_raw_node());

      depthmap_tf_filter_->connectInput(*depthmap_sub_);

      // subscribe to CameraInfo  topic
      std::string info_topic = image_transport::getCameraInfoTopic(depthmap_topic);

      rclcpp::SubscriptionOptions sub_opts;
      sub_opts.event_callbacks.message_lost_callback =
        [&](rclcpp::QOSMessageLostInfo & info)
        {
          std::ostringstream sstm;
          sstm << "Some messages were lost:\n>\tNumber of new lost messages: " <<
            info.total_count_change << " \n>\tTotal number of messages lost: " <<
            info.total_count;
          setStatus(
            rviz_common::properties::StatusProperty::Warn,
            "Depth Camera Info",
            QString(sstm.str().c_str()));
        };

      cam_info_sub_ = rviz_ros_node_->get_raw_node()->
        create_subscription<sensor_msgs::msg::CameraInfo>(
        info_topic,
        rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(cam_info_mutex_);
          cam_info_ = msg;
        }, sub_opts);

      if (!color_topic.empty() && !color_transport.empty()) {
        // subscribe to color image topic
        rgb_sub_->subscribe(
          rviz_ros_node_->get_raw_node().get(),
          color_topic, color_transport, rclcpp::SensorDataQoS().get_rmw_qos_profile());

        // connect message filters to synchronizer
        sync_depth_color_->connectInput(*depthmap_tf_filter_, *rgb_sub_);
        sync_depth_color_->setInterMessageLowerBound(0, rclcpp::Duration(0, 0.5 * 1e+9));
        sync_depth_color_->setInterMessageLowerBound(1, rclcpp::Duration(0, 0.5 * 1e+9));
        sync_depth_color_->registerCallback(
          std::bind(
            &DepthCloudDisplay::processMessage, this,
            std::placeholders::_1, std::placeholders::_2));

        pointcloud_common_->color_transformer_property_->setValue("RGB8");
      } else {
        depthmap_tf_filter_->registerCallback(
          std::bind(&DepthCloudDisplay::processDepthMessage, this, std::placeholders::_1));
      }
    }
  } catch (const image_transport::TransportLoadException & e) {
    setStatus(
      rviz_common::properties::StatusProperty::Error, "Message",
      QString("Error subscribing: ") + e.what());
  }
}

void DepthCloudDisplay::caminfoCallback(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(cam_info_mutex_);
  cam_info_ = std::move(msg);
}

void DepthCloudDisplay::unsubscribe()
{
  clear();

  sync_depth_color_.reset(new SynchronizerDepthColor(SyncPolicyDepthColor(queue_size_)));
  depthmap_tf_filter_.reset();
  depthmap_sub_.reset();
  rgb_sub_.reset();
  cam_info_sub_.reset();
}

void DepthCloudDisplay::clear()
{
  pointcloud_common_->reset();
}

void DepthCloudDisplay::update(float wall_dt, float ros_dt)
{
  pointcloud_common_->update(wall_dt, ros_dt);
}

void DepthCloudDisplay::reset()
{
  clear();
  messages_received_ = 0;
  setStatus(rviz_common::properties::StatusProperty::Ok, "Depth Map", "0 depth maps received");
  setStatus(rviz_common::properties::StatusProperty::Ok, "Message", "Ok");
}

void DepthCloudDisplay::processDepthMessage(const sensor_msgs::msg::Image::ConstSharedPtr depth_msg)
{
  processMessage(depth_msg, sensor_msgs::msg::Image::ConstSharedPtr());
}

void DepthCloudDisplay::processMessage(
  const sensor_msgs::msg::Image::ConstSharedPtr depth_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr rgb_msg)
{
  if (context_->getFrameManager()->getPause()) {
    return;
  }

  std::ostringstream s;

  ++messages_received_;
  setStatus(
    rviz_common::properties::StatusProperty::Ok, "Depth Map",
    QString::number(messages_received_) + " depth maps received");
  setStatus(rviz_common::properties::StatusProperty::Ok, "Message", "Ok");

  sensor_msgs::msg::CameraInfo::ConstSharedPtr cam_info;
  {
    std::lock_guard<std::mutex> lock(cam_info_mutex_);
    cam_info = cam_info_;
  }

  if (!cam_info || !depth_msg) {
    return;
  }

  s.str("");
  s << depth_msg->width << " x " << depth_msg->height;
  setStatusStd(rviz_common::properties::StatusProperty::Ok, "Depth Image Size", s.str());

  if (rgb_msg) {
    s.str("");
    s << rgb_msg->width << " x " << rgb_msg->height;
    setStatusStd(rviz_common::properties::StatusProperty::Ok, "Image Size", s.str());

    if (depth_msg->header.frame_id != rgb_msg->header.frame_id) {
      std::stringstream errorMsg;
      errorMsg << "Depth image frame id [" << depth_msg->header.frame_id.c_str()
               << "] doesn't match color image frame id ["
               << rgb_msg->header.frame_id.c_str() << "]";
      setStatusStd(rviz_common::properties::StatusProperty::Warn, "Message", errorMsg.str());
    }
  }

  if (use_auto_size_property_->getBool()) {
    float f = cam_info->k[0];
    float bx = cam_info->binning_x > 0 ? cam_info->binning_x : 1.0;
    float s = auto_size_factor_property_->getFloat();
    pointcloud_common_->point_world_size_property_->setFloat(s / f * bx);
  }

  bool use_occlusion_compensation = use_occlusion_compensation_property_->getBool();

  if (use_occlusion_compensation) {
    // reset depth cloud display if camera moves
    Ogre::Quaternion orientation;
    Ogre::Vector3 position;

    if (!context_->getFrameManager()->getTransform(depth_msg->header, position, orientation)) {
      setStatus(
        rviz_common::properties::StatusProperty::Error, "Message",
        QString("Failed to transform from frame [") + depth_msg->header.frame_id.c_str() +
        QString("] to frame [") + context_->getFrameManager()->getFixedFrame().c_str() +
        QString("]"));
      return;
    } else {
      Ogre::Radian angle;
      Ogre::Vector3 axis;

      (current_orientation_.Inverse() * orientation).ToAngleAxis(angle, axis);

      float angle_deg = angle.valueDegrees();
      if (angle_deg >= 180.0f) {
        angle_deg -= 180.0f;
      }
      if (angle_deg < -180.0f) {
        angle_deg += 180.0f;
      }

      if (trans_thres_ == 0.0 || angular_thres_ == 0.0 ||
        (position - current_position_).length() > trans_thres_ || angle_deg > angular_thres_)
      {
        // camera orientation/position changed
        current_position_ = position;
        current_orientation_ = orientation;

        // reset multi-layered depth image
        ml_depth_data_->reset();
      }
    }
  }

  try {
    auto rviz_ros_node_ = context_->getRosNodeAbstraction().lock();

    sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg =
      ml_depth_data_->generatePointCloudFromDepth(depth_msg, rgb_msg, cam_info, rviz_ros_node_);

    if (!cloud_msg.get()) {
      std::ostringstream sstm;
      sstm << "generatePointCloudFromDepth() returned zero.";
      setStatus(
        rviz_common::properties::StatusProperty::Warn,
        "Depth Camera Info",
        QString(sstm.str().c_str()));
    }
    cloud_msg->header = depth_msg->header;

    // add point cloud message to pointcloud_common to be visualized
    pointcloud_common_->addMessage(cloud_msg);
  } catch (rviz_common::MultiLayerDepthException & e) {
    setStatus(
      rviz_common::properties::StatusProperty::Error, "Message",
      QString("Error updating depth cloud: ") + e.what());
  }
}


void DepthCloudDisplay::scanForTransportSubscriberPlugins()
{
  pluginlib::ClassLoader<image_transport::SubscriberPlugin> sub_loader(
    "image_transport", "image_transport::SubscriberPlugin");

  std::vector<std::string> lookup_names = sub_loader.getDeclaredClasses();
  for (const auto & lookup_name : lookup_names) {
    // lookup_name is formatted as "pkg/transport_sub", for instance
    // "image_transport/compressed_sub" for the "compressed"
    // transport.  This code removes the "_sub" from the tail and
    // everything up to and including the "/" from the head, leaving
    // "compressed" (for example) in transport_name.
    std::string transport_name = image_transport::erase_last_copy(lookup_name, "_sub");
    transport_name = transport_name.substr(lookup_name.find('/') + 1);

    // If the plugin loads without throwing an exception, add its
    // transport name to the list of valid plugins, otherwise ignore
    // it.
    try {
      std::shared_ptr<image_transport::SubscriberPlugin> sub =
        sub_loader.createSharedInstance(lookup_name);
      transport_plugin_types_.insert(transport_name);
    } catch (const pluginlib::LibraryLoadException & /*e*/) {
    } catch (const pluginlib::CreateClassException & /*e*/) {
    }
  }
}

void DepthCloudDisplay::updateTopic()
{
  unsubscribe();
  reset();
  subscribe();
  context_->queueRender();
}

void DepthCloudDisplay::fillTransportOptionList(rviz_common::properties::EnumProperty * property)
{
  property->clearOptions();

  std::vector<std::string> choices;

  choices.push_back("raw");

  auto rviz_ros_node_ = context_->getRosNodeAbstraction().lock();

  std::map<std::string, std::vector<std::string>> published_topics =
    rviz_ros_node_->get_topic_names_and_types();

  const std::string & topic = depth_topic_property_->getStdString();

  for (const auto &[topic_name, topic_types] : published_topics) {
    if (topic_name.find(topic) == 0 && topic_name != topic && topic_name[topic.size()] == '/' &&
      topic_name.find('/', topic.size() + 1) == std::string::npos)
    {
      std::string transport_type = topic_name.substr(topic.size() + 1);

      // If the transport type string found above is in the set of
      // supported transport type plugins, add it to the list.
      if (transport_plugin_types_.find(transport_type) != transport_plugin_types_.end()) {
        choices.push_back(transport_type);
      }
    }
  }

  for (size_t i = 0; i < choices.size(); i++) {
    property->addOptionStd(choices[i]);
  }
}

void DepthCloudDisplay::fixedFrameChanged()
{
  Display::reset();
}

}  // namespace displays
}  // namespace rviz_default_plugins

#include <pluginlib/class_list_macros.hpp>   // NOLINT: cpplint cannot handle the include order here

PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::DepthCloudDisplay, rviz_common::Display)
