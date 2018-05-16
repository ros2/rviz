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

#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__MARKER__MARKER_DISPLAY_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__MARKER__MARKER_DISPLAY_HPP_

#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>
#include <utility>

#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "rviz_common/display.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/ros_topic_display.hpp"
#include "rviz_common/interaction/forwards.hpp"

namespace rviz_common
{
class QueueSizeProperty;

namespace properties
{
class IntProperty;
}
}

using rviz_common::properties::StatusLevel;

namespace rviz_default_plugins
{
namespace displays
{
class MarkerNamespace;

namespace markers
{
class MarkerBase;
class MarkerSelectionHandler;
class MarkerFactory;
}

typedef std::shared_ptr<markers::MarkerSelectionHandler> MarkerSelectionHandlerPtr;
typedef std::shared_ptr<markers::MarkerBase> MarkerBasePtr;
typedef std::pair<std::string, int32_t> MarkerID;

/**
 * \class MarkerDisplay
 * \brief Displays "markers" sent in by other ROS nodes on the "visualization_marker" topic
 *
 * Markers come in as visualization_msgs::msg::Marker messages.
 * See the Marker message for more information.
 */
class MarkerDisplay : public rviz_common::RosTopicDisplay<visualization_msgs::msg::Marker>
{
  Q_OBJECT

public:
  // TODO(Martin-Idel-SI): Constructor for testing, remove once ros_nodes can be mocked and call
  // initialize instead
  explicit MarkerDisplay(
    std::unique_ptr<markers::MarkerFactory> factory,
    rviz_common::DisplayContext * display_context);
  MarkerDisplay();
  ~MarkerDisplay() override;

  void onInitialize() override;

  void update(float wall_dt, float ros_dt) override;

  void fixedFrameChanged() override;
  void reset() override;

  void deleteMarker(MarkerID id);

  /** @brief Delete all known markers to this plugin, regardless of id or namespace **/
  void deleteAllMarkers();

  void setMarkerStatus(MarkerID id, StatusLevel level, const std::string & text);
  void deleteMarkerStatus(MarkerID id);

  /**
   * \brief Processes a marker message
   * @param message The message to process
   */
  void processMessage(visualization_msgs::msg::Marker::ConstSharedPtr message) override;

protected:
  void onEnable() override;
  void onDisable() override;

  void load(const rviz_common::Config & config) override;

  /** @brief Subscribes to the "visualization_marker" and
   * "visualization_marker_array" topics. */
  void subscribe() override;

  /** @brief Unsubscribes from the "visualization_marker"
   * "visualization_marker_array" topics. */
  void unsubscribe() override;

  /** @brief Process a MarkerArray message. */
  void incomingMarkerArray(visualization_msgs::msg::MarkerArray::ConstSharedPtr array);

  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr array_sub_;
  std::unique_ptr<rviz_common::QueueSizeProperty> queue_size_property_;

private:
  /** @brief Delete all the markers within the given namespace. */
  void deleteMarkersInNamespace(const std::string & ns);

  /**
   * \brief Removes all the markers
   */
  void clearMarkers();

  /**
   * \brief Processes an "Add" marker message
   * @param message The message to process
   */
  void processAdd(visualization_msgs::msg::Marker::ConstSharedPtr message);
  /**
   * \brief Processes a "Delete" marker message
   * @param message The message to process
   */
  void processDelete(visualization_msgs::msg::Marker::ConstSharedPtr message);

  /**
   * \brief ROS callback notifying us of a new marker
   */
  void incomingMarker(visualization_msgs::msg::Marker::ConstSharedPtr marker);

  void createMarkerArraySubscription();

  typedef std::vector<visualization_msgs::msg::Marker::ConstSharedPtr> V_MarkerMessage;
  V_MarkerMessage takeSnapshotOfMessageQueue();
  void processNewMessages(const V_MarkerMessage & local_queue);
  void removeExpiredMarkers();

  void updateMarkersWithLockedFrame() const;
  QHash<QString, MarkerNamespace *>::const_iterator getMarkerNamespace(
    const visualization_msgs::msg::Marker::ConstSharedPtr & message);
  MarkerBasePtr createOrGetOldMarker(
    const visualization_msgs::msg::Marker::ConstSharedPtr & message);
  MarkerBasePtr createMarker(const visualization_msgs::msg::Marker::ConstSharedPtr & message);
  void configureMarker(
    const visualization_msgs::msg::Marker::ConstSharedPtr & message, MarkerBasePtr & marker);

  typedef std::map<MarkerID, MarkerBasePtr> M_IDToMarker;
  typedef std::set<MarkerBasePtr> S_MarkerBase;
  M_IDToMarker markers_;                  ///< Map of marker id to the marker info structure
  S_MarkerBase markers_with_expiration_;
  S_MarkerBase frame_locked_markers_;
  ///< Marker message queue.  Messages are added to this as they are received, and then processed
  ///< in our update() function
  V_MarkerMessage message_queue_;
  std::mutex queue_mutex_;

  typedef QHash<QString, MarkerNamespace *> M_Namespace;
  M_Namespace namespaces_;

  rviz_common::properties::Property * namespaces_category_;

  typedef std::map<QString, bool> M_EnabledState;
  M_EnabledState namespace_config_enabled_state_;

  std::unique_ptr<markers::MarkerFactory> marker_factory_;

  friend class MarkerNamespace;
};

/** @brief Manager of a single marker namespace.  Keeps a hash from
 * marker IDs to MarkerBasePtr, and creates or destroys them when necessary. */
class MarkerNamespace : public rviz_common::properties::BoolProperty
{
  Q_OBJECT

public:
  MarkerNamespace(
    const QString & name,
    rviz_common::properties::Property * parent_property,
    MarkerDisplay * owner
  );
  bool isEnabled() const {return getBool();}

public Q_SLOTS:
  void onEnableChanged();

private:
  MarkerDisplay * owner_;
};

}  // namespace displays
}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__MARKER__MARKER_DISPLAY_HPP_
