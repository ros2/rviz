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

#include "rviz_default_plugins/displays/marker/marker_common.hpp"

#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/duration.hpp"

#include "rviz_common/display.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/properties/property.hpp"
#include "rviz_common/validate_floats.hpp"

#include "rviz_default_plugins/displays/marker/markers/marker_factory.hpp"

namespace rviz_default_plugins
{
namespace displays
{

MarkerCommon::MarkerCommon(rviz_common::Display * display)
: display_(display)
{
  namespaces_category_ = new rviz_common::properties::Property(
    "Namespaces", QVariant(), "", display_);
  marker_factory_ = std::make_unique<markers::MarkerFactory>();
}

MarkerCommon::~MarkerCommon()
{
  clearMarkers();
}

void MarkerCommon::initialize(rviz_common::DisplayContext * context, Ogre::SceneNode * scene_node)
{
  context_ = context;
  scene_node_ = scene_node;

  namespace_config_enabled_state_.clear();

  marker_factory_->initialize(this, context_, scene_node_);
}

void MarkerCommon::load(const rviz_common::Config & config)
{
  rviz_common::Config c = config.mapGetChild("Namespaces");
  for (rviz_common::Config::MapIterator iter = c.mapIterator(); iter.isValid(); iter.advance() ) {
    QString key = iter.currentKey();
    const rviz_common::Config & child = iter.currentChild();
    namespace_config_enabled_state_[key] = child.getValue().toBool();
  }
}

void MarkerCommon::clearMarkers()
{
  markers_.clear();
  markers_with_expiration_.clear();
  frame_locked_markers_.clear();
  namespaces_category_->removeChildren();
  namespaces_.clear();
}

void MarkerCommon::deleteMarker(MarkerID id)
{
  deleteMarkerStatus(id);

  auto it = markers_.find(id);
  if (it != markers_.end()) {
    markers_with_expiration_.erase(it->second);
    frame_locked_markers_.erase(it->second);
    markers_.erase(it);
  }
}

void MarkerCommon::deleteMarkersInNamespace(const std::string & ns)
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

void MarkerCommon::deleteAllMarkers()
{
  std::vector<MarkerID> to_delete;
  for (auto const & marker : markers_) {
    to_delete.push_back(marker.first);
  }

  for (auto & marker : to_delete) {
    deleteMarker(marker);
  }
}

void MarkerCommon::setMarkerStatus(MarkerID id, StatusLevel level, const std::string & text)
{
  std::string marker_name = id.first + "/" + std::to_string(id.second);
  display_->setStatusStd(level, marker_name, text);
}

void MarkerCommon::deleteMarkerStatus(MarkerID id)
{
  std::string marker_name = id.first + "/" + std::to_string(id.second);
  display_->deleteStatusStd(marker_name);
}

void MarkerCommon::addMessage(const visualization_msgs::msg::Marker::ConstSharedPtr marker)
{
  std::unique_lock<std::mutex> lock(queue_mutex_);

  message_queue_.push_back(marker);
}

void MarkerCommon::addMessage(
  const visualization_msgs::msg::MarkerArray::ConstSharedPtr array)
{
  using ns_type = decltype(visualization_msgs::msg::Marker::ns);
  using id_type = decltype(visualization_msgs::msg::Marker::id);
  using pair_type = std::pair<id_type, const ns_type &>;

  // Keep track of unique markers
  std::set<pair_type> unique_markers;
  bool found_duplicate = false;
  std::string offending_ns;
  id_type offending_id = 0;

  for (auto const & marker : array->markers) {
    if (!found_duplicate) {
      pair_type pair(marker.id, marker.ns);
      found_duplicate = !unique_markers.insert(pair).second;
      if (found_duplicate) {
        offending_ns = marker.ns;
        offending_id = marker.id;
      }
    }
    addMessage(std::make_shared<visualization_msgs::msg::Marker>(marker));
  }

  // Can't use setMarkerStatus on individual markers because processAdd would clear it.
  const char * kDuplicateStatus = "Duplicate Marker Check";
  if (found_duplicate) {
    std::stringstream error_stream;
    error_stream << "Multiple Markers in the same MarkerArray message had the same ns and id: ";
    error_stream << "(" << offending_ns << ", " << offending_id << ")";
    display_->setStatusStd(
      rviz_common::properties::StatusProperty::Error,
      kDuplicateStatus,
      error_stream.str());
  } else {
    display_->deleteStatusStd(kDuplicateStatus);
  }
}

bool validateFloats(const visualization_msgs::msg::Marker & msg)
{
  bool valid = true;
  valid = valid && rviz_common::validateFloats(msg.pose);
  valid = valid && rviz_common::validateFloats(msg.scale);
  valid = valid && rviz_common::validateFloats(msg.color);
  valid = valid && rviz_common::validateFloats(msg.points);
  return valid;
}

void MarkerCommon::processMessage(const visualization_msgs::msg::Marker::ConstSharedPtr message)
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
      if (!message->ns.empty()) {
        deleteMarkersInNamespace(message->ns);
      } else {
        deleteAllMarkers();
      }
      break;

    default:
      RVIZ_COMMON_LOG_ERROR_STREAM("Unknown marker action: " << message->action);
  }
}

QHash<QString, MarkerNamespace *>::const_iterator MarkerCommon::getMarkerNamespace(
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

void MarkerCommon::processAdd(const visualization_msgs::msg::Marker::ConstSharedPtr message)
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

MarkerBasePtr MarkerCommon::createOrGetOldMarker(
  const visualization_msgs::msg::Marker::ConstSharedPtr & message)
{
  MarkerBasePtr marker;
  auto it = markers_.find(MarkerID(message->ns, message->id));
  if (it != markers_.end()) {
    marker = it->second;
    markers_with_expiration_.erase(marker);
    frame_locked_markers_.erase(marker);
    if (message->type != marker->getMessage()->type) {
      markers_.erase(it);
      marker = createMarker(message);
    }
  } else {
    marker = createMarker(message);
  }
  return marker;
}

MarkerBasePtr MarkerCommon::createMarker(
  const visualization_msgs::msg::Marker::ConstSharedPtr & message)
{
  auto marker = marker_factory_->createMarkerForType(message->type);
  markers_.insert(make_pair(MarkerID(message->ns, message->id), marker));
  return marker;
}

void MarkerCommon::configureMarker(
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

void MarkerCommon::processDelete(const visualization_msgs::msg::Marker::ConstSharedPtr message)
{
  deleteMarker(MarkerID(message->ns, message->id));

  context_->queueRender();
}

void MarkerCommon::update(float wall_dt, float ros_dt)
{
  (void) wall_dt;
  (void) ros_dt;

  MarkerCommon::V_MarkerMessage local_queue = takeSnapshotOfMessageQueue();
  processNewMessages(local_queue);
  removeExpiredMarkers();
  updateMarkersWithLockedFrame();
}

MarkerCommon::V_MarkerMessage MarkerCommon::takeSnapshotOfMessageQueue()
{
  std::unique_lock<std::mutex> lock(queue_mutex_);

  V_MarkerMessage local_queue;
  local_queue.swap(message_queue_);

  return local_queue;
}

void MarkerCommon::processNewMessages(const MarkerCommon::V_MarkerMessage & local_queue)
{
  if (!local_queue.empty()) {
    for (auto const & message : local_queue) {
      processMessage(message);
    }
  }
}

void MarkerCommon::removeExpiredMarkers()
{
  std::vector<MarkerBasePtr> markers_to_delete;
  for (const auto & marker : markers_with_expiration_) {
    if (marker->expired()) {
      markers_to_delete.push_back(marker);
    }
  }
  for (const auto & marker : markers_to_delete) {
    deleteMarker(marker->getID());
  }
}

void MarkerCommon::updateMarkersWithLockedFrame() const
{
  for (auto const & locked_marker : frame_locked_markers_) {
    locked_marker->updateFrameLocked();
  }
}

MarkerNamespace::MarkerNamespace(
  const QString & name, rviz_common::properties::Property * parent_property, MarkerCommon * owner)
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
