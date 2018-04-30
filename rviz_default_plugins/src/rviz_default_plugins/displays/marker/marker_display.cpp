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

#include "marker_display.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/duration.hpp"

#include "rviz_rendering/objects/arrow.hpp"
#include "rviz_rendering/objects/billboard_line.hpp"
#include "rviz_rendering/objects/shape.hpp"
#include "rviz_common/properties/int_property.hpp"
#include "rviz_common/properties/queue_size_property.hpp"
#include "rviz_common/validate_floats.hpp"

#include "markers/arrow_marker.hpp"
#include "markers/line_list_marker.hpp"
#include "markers/line_strip_marker.hpp"
#include "markers/mesh_resource_marker.hpp"
#include "markers/points_marker.hpp"
#include "markers/shape_marker.hpp"
#include "markers/text_view_facing_marker.hpp"
#include "markers/triangle_list_marker.hpp"
#include "markers/marker_factory.hpp"

namespace rviz_default_plugins
{
namespace displays
{

MarkerDisplay::MarkerDisplay(
  std::unique_ptr<markers::MarkerFactory> factory, rviz_common::DisplayContext * display_context)
: MarkerDisplay()
{
  marker_factory_ = std::move(factory);
  context_ = display_context;
}

MarkerDisplay::MarkerDisplay()
: queue_size_property_(std::make_unique<rviz_common::QueueSizeProperty>(this, 10))
{
  topic_property_->setDescription(
    "visualization_msgs::msg::Marker topic to subscribe to. <topic>_array will also"
    " automatically be subscribed with type visualization_msgs::msg::MarkerArray.");

  namespaces_category_ = new rviz_common::properties::Property("Namespaces", QVariant(), "", this);
  marker_factory_ = std::make_unique<markers::MarkerFactory>();
}

void MarkerDisplay::onInitialize()
{
  RTDClass::onInitialize();
  namespace_config_enabled_state_.clear();

  marker_factory_->initialize(this, context_, scene_node_);

  // TODO(greimela): Revisit after MessageFilter is available in ROS2
//  tf_filter_ = new tf::MessageFilter<visualization_msgs::Marker>( *context_->getTFClient(),
//    fixed_frame_.toStdString(),
//    queue_size_property_->getInt(),
//    update_nh_ );
//
//  tf_filter_->connectInput(sub_);
//  tf_filter_->registerCallback(boost::bind(&MarkerDisplay::incomingMarker, this, _1));
//  tf_filter_->registerFailureCallback(boost::bind(&MarkerDisplay::failedMarker, this, _1, _2));
}

MarkerDisplay::~MarkerDisplay()
{
  if (initialized()) {
    unsubscribe();
    clearMarkers();
  }
}

void MarkerDisplay::load(const rviz_common::Config & config)
{
  rviz_common::Display::load(config);

  rviz_common::Config c = config.mapGetChild("Namespaces");
  for (rviz_common::Config::MapIterator iter = c.mapIterator(); iter.isValid(); iter.advance() ) {
    QString key = iter.currentKey();
    const rviz_common::Config & child = iter.currentChild();
    namespace_config_enabled_state_[key] = child.getValue().toBool();
  }
}

void MarkerDisplay::clearMarkers()
{
  markers_.clear();
  markers_with_expiration_.clear();
  frame_locked_markers_.clear();
  namespaces_category_->removeChildren();
  namespaces_.clear();
}

void MarkerDisplay::onEnable()
{
  subscribe();
}

void MarkerDisplay::onDisable()
{
  unsubscribe();
  clearMarkers();
}

void MarkerDisplay::subscribe()
{
  RTDClass::subscribe();

  if ((!isEnabled()) || (topic_property_->getTopicStd().empty())) {
    return;
  }

  createMarkerArraySubscription();
}

void MarkerDisplay::createMarkerArraySubscription()
{
  try {
    array_sub_ = node_->create_subscription<visualization_msgs::msg::MarkerArray>(
      topic_property_->getTopicStd() + "_array",
      [this](visualization_msgs::msg::MarkerArray::ConstSharedPtr msg) {
        incomingMarkerArray(msg);
      },
      qos_profile);
    setStatus(StatusLevel::Ok, "Array Topic", "OK");
  } catch (rclcpp::exceptions::InvalidTopicNameError & e) {
    setStatus(StatusLevel::Error, "Array Topic", QString("Error subscribing: ") + e.what());
  }
}

void MarkerDisplay::unsubscribe()
{
  RTDClass::unsubscribe();
  array_sub_.reset();
}

void MarkerDisplay::deleteMarker(MarkerID id)
{
  deleteMarkerStatus(id);

  auto it = markers_.find(id);
  if (it != markers_.end()) {
    markers_with_expiration_.erase(it->second);
    frame_locked_markers_.erase(it->second);
    markers_.erase(it);
  }
}

void MarkerDisplay::deleteMarkersInNamespace(const std::string & ns)
{
  std::vector<MarkerID> to_delete;

  // TODO(anon): this is inefficient, should store every in-use id per namespace and lookup by that
  for (auto const & marker : markers_) {
    if (marker.first.first == ns) {
      to_delete.push_back(marker.first);
    }
  }

  for (auto & marker : to_delete) {
    deleteMarker(marker);
  }
}

void MarkerDisplay::deleteAllMarkers()
{
  std::vector<MarkerID> to_delete;
  for (auto const & marker : markers_) {
    to_delete.push_back(marker.first);
  }

  for (auto & marker : to_delete) {
    deleteMarker(marker);
  }
}

void MarkerDisplay::setMarkerStatus(MarkerID id, StatusLevel level, const std::string & text)
{
  std::string marker_name = id.first + "/" + std::to_string(id.second);
  setStatusStd(level, marker_name, text);
}

void MarkerDisplay::deleteMarkerStatus(MarkerID id)
{
  std::string marker_name = id.first + "/" + std::to_string(id.second);
  deleteStatusStd(marker_name);
}

void MarkerDisplay::incomingMarkerArray(
  const visualization_msgs::msg::MarkerArray::ConstSharedPtr array)
{
  for (auto const & marker : array->markers) {
    incomingMarker(std::make_shared<visualization_msgs::msg::Marker>(marker));
  }
}

void MarkerDisplay::incomingMarker(const visualization_msgs::msg::Marker::ConstSharedPtr marker)
{
  std::unique_lock<std::mutex> lock(queue_mutex_);

  message_queue_.push_back(marker);
}

// TODO(greimela): Revisit after MessageFilter is available in ROS2
// void MarkerDisplay::failedMarker(const ros::MessageEvent<visualization_msgs::Marker>&
// marker_evt, tf::FilterFailureReason reason)
// {
//  visualization_msgs::Marker::ConstPtr marker = marker_evt.getConstMessage();
//  if (marker->action == visualization_msgs::msg::Marker::DELETE ||
//      marker->action == visualization_msgs::msg::Marker::DELETEALL)
//  {
//    return this->processMessage(marker);
//  }
//  std::string authority = marker_evt.getPublisherName();
//  std::string error = context_->getFrameManager()
//    ->discoverFailureReason(marker->header.frame_id, marker->header.stamp, authority, reason);
//  setMarkerStatus(MarkerID(marker->ns, marker->id), StatusProperty::Error, error);
// }

bool validateFloats(const visualization_msgs::msg::Marker & msg)
{
  bool valid = true;
  valid = valid && rviz_common::validateFloats(msg.pose);
  valid = valid && rviz_common::validateFloats(msg.scale);
  valid = valid && rviz_common::validateFloats(msg.color);
  valid = valid && rviz_common::validateFloats(msg.points);
  return valid;
}

void MarkerDisplay::processMessage(const visualization_msgs::msg::Marker::ConstSharedPtr message)
{
  if (!validateFloats(*message)) {
    setMarkerStatus(
      MarkerID(message->ns, message->id),
      rviz_common::properties::StatusProperty::Error,
      "Contains invalid floating point values (nans or infs)");
    return;
  }

  switch (message->action) {
    case visualization_msgs::msg::Marker::ADD:
      processAdd(message);
      break;

    case visualization_msgs::msg::Marker::DELETE:
      processDelete(message);
      break;

    case visualization_msgs::msg::Marker::DELETEALL:
      deleteAllMarkers();
      break;

    default:
      RVIZ_COMMON_LOG_ERROR_STREAM("Unknown marker action: " << message->action);
  }
}

QHash<QString, MarkerNamespace *>::const_iterator MarkerDisplay::getMarkerNamespace(
  const visualization_msgs::msg::Marker::ConstSharedPtr & message)
{
  QString namespace_name = QString::fromStdString(message->ns);
  auto ns_it = namespaces_.find(namespace_name);
  if (ns_it == namespaces_.end() ) {
    ns_it = namespaces_.insert(
      namespace_name, new MarkerNamespace(namespace_name, namespaces_category_, this));

    // Adding a new namespace, determine if it's configured to be disabled
    if (namespace_config_enabled_state_.count(namespace_name) > 0 &&
      !namespace_config_enabled_state_[namespace_name])
    {
      ns_it.value()->setValue(false);  // Disable the namespace
    }
  }
  return ns_it;
}

void MarkerDisplay::processAdd(const visualization_msgs::msg::Marker::ConstSharedPtr message)
{
  auto ns_it = getMarkerNamespace(message);

  if (!ns_it.value()->isEnabled() ) {
    return;
  }

  deleteMarkerStatus(MarkerID(message->ns, message->id));

  MarkerBasePtr marker = createOrGetOldMarker(message);

  if (marker) {
    configureMarker(message, marker);
  }
}

MarkerBasePtr MarkerDisplay::createOrGetOldMarker(
  const visualization_msgs::msg::Marker::ConstSharedPtr & message)
{
  MarkerBasePtr marker;
  auto it = markers_.find(MarkerID(message->ns, message->id));
  if (it != markers_.end()) {
    marker = it->second;
    markers_with_expiration_.erase(marker);
    if (message->type != marker->getMessage()->type) {
      markers_.erase(it);
      marker = createMarker(message);
    }
  } else {
    marker = createMarker(message);
  }
  return marker;
}

MarkerBasePtr MarkerDisplay::createMarker(
  const visualization_msgs::msg::Marker::ConstSharedPtr & message)
{
  auto marker = marker_factory_->createMarkerForType(message->type);
  markers_.insert(make_pair(MarkerID(message->ns, message->id), marker));
  return marker;
}

void MarkerDisplay::configureMarker(
  const visualization_msgs::msg::Marker::ConstSharedPtr & message, MarkerBasePtr & marker)
{
  marker->setMessage(message);

  if (rclcpp::Duration(message->lifetime).nanoseconds() > 100000) {
    markers_with_expiration_.insert(marker);
  }

  if (message->frame_locked) {
    frame_locked_markers_.insert(marker);
  }

  context_->queueRender();
}

void MarkerDisplay::processDelete(const visualization_msgs::msg::Marker::ConstSharedPtr message)
{
  deleteMarker(MarkerID(message->ns, message->id));

  context_->queueRender();
}

void MarkerDisplay::update(float wall_dt, float ros_dt)
{
  (void) wall_dt;
  (void) ros_dt;

  MarkerDisplay::V_MarkerMessage local_queue = takeSnapshotOfMessageQueue();
  processNewMessages(local_queue);
  removeExpiredMarkers();
  updateMarkersWithLockedFrame();
}

MarkerDisplay::V_MarkerMessage MarkerDisplay::takeSnapshotOfMessageQueue()
{
  std::unique_lock<std::mutex> lock(queue_mutex_);

  V_MarkerMessage local_queue;
  local_queue.swap(message_queue_);

  return local_queue;
}

void MarkerDisplay::processNewMessages(const MarkerDisplay::V_MarkerMessage & local_queue)
{
  if (!local_queue.empty()) {
    for (auto const & message : local_queue) {
      processMessage(message);
    }
  }
}

void MarkerDisplay::removeExpiredMarkers()
{
  auto marker_it = markers_with_expiration_.begin();
  auto end = markers_with_expiration_.end();
  for (; marker_it != end; ) {
    MarkerBasePtr marker = *marker_it;
    if (marker->expired()) {
      ++marker_it;
      deleteMarker(marker->getID());
    } else {
      ++marker_it;
    }
  }
}

void MarkerDisplay::updateMarkersWithLockedFrame() const
{
  for (auto const & locked_marker : frame_locked_markers_) {
    locked_marker->updateFrameLocked();
  }
}

void MarkerDisplay::fixedFrameChanged()
{
  RTDClass::fixedFrameChanged();
  clearMarkers();
}

void MarkerDisplay::reset()
{
  RTDClass::reset();
  clearMarkers();
}

MarkerNamespace::MarkerNamespace(
  const QString & name, rviz_common::properties::Property * parent_property, MarkerDisplay * owner)
: BoolProperty(name, true, "Enable/disable all markers in this namespace.", parent_property),
  owner_(owner)
{
  // Can't do this connect in chained constructor above because at
  // that point it doesn't really know that "this" is a
  // MarkerNamespace*, so the signal doesn't get connected.
  connect(this, SIGNAL(changed()), this, SLOT(onEnableChanged()));
}

void MarkerNamespace::onEnableChanged()
{
  if (!isEnabled()) {
    owner_->deleteMarkersInNamespace(getName().toStdString());
  }

  // Update the configuration that stores the enabled state of all markers
  owner_->namespace_config_enabled_state_[getName()] = isEnabled();
}

}  // namespace displays
}  // namespace rviz_default_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::MarkerDisplay, rviz_common::Display)
