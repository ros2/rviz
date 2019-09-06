/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * Copyright (c) 2019, Open Source Robotics Foundation, Inc.
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
 *     * Neither the name of the copyright holder nor the names of its
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
#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__INTERACTIVE_MARKERS__INTERACTIVE_MARKER_DISPLAY_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__INTERACTIVE_MARKERS__INTERACTIVE_MARKER_DISPLAY_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_update.hpp"
#include "visualization_msgs/msg/interactive_marker_init.hpp"

#ifndef Q_MOC_RUN
#include "interactive_markers/interactive_marker_client.hpp"
#endif

#include "rviz_common/display.hpp"

#include "rviz_default_plugins/displays/interactive_markers/interactive_marker.hpp"

namespace rviz_common
{
class BoolProperty;
class Object;
}

namespace rviz_default_plugins
{
class InteractiveMarkerNamespaceProperty;

namespace displays
{
class MarkerBase;

/// Displays Interactive Markers
class InteractiveMarkerDisplay : public rviz_common::Display
{
  Q_OBJECT

public:
  InteractiveMarkerDisplay();

  // Overrides from Display
  void update(float wall_dt, float ros_dt) override;

  void reset() override;

protected:
  // Overrides from Display
  void fixedFrameChanged() override;

  void onInitialize() override;

  void onEnable() override;

  void onDisable() override;

protected Q_SLOTS:
  void namespaceChanged();
  void updateShowDescriptions();
  void updateShowAxes();
  void updateShowVisualAids();
  void updateEnableTransparency();
  void publishFeedback(visualization_msgs::msg::InteractiveMarkerFeedback & feedback);
  void onStatusUpdate(
    rviz_common::properties::StatusProperty::Level level,
    const std::string & name,
    const std::string & text);

private:
  /// Subscribe to all message topics.
  void subscribe();

  /// Unsubscribe from all message topics.
  void unsubscribe();

  /// Called by InteractiveMarkerClient when successfully initialized.
  void initializeCallback(visualization_msgs::srv::GetInteractiveMarkers::Response::SharedPtr);

  /// Called by InteractiveMarkerClient when an update from a server is received.
  void updateCallback(visualization_msgs::msg::InteractiveMarkerUpdate::ConstSharedPtr msg);

  /// Called by InteractiveMarkerClient when it resets.
  void resetCallback();

  /// Called by InteractiveMarkerClient when there is a status message.
  void statusCallback(
    interactive_markers::InteractiveMarkerClient::Status,
    const std::string & message);

  void updateMarkers(const std::vector<visualization_msgs::msg::InteractiveMarker> & markers);

  void updatePoses(
    const std::vector<visualization_msgs::msg::InteractiveMarkerPose> & marker_poses);

  /// Erase all visualization markers.
  void eraseAllMarkers();

  /// Erase visualization markers for an InteractionMarkerServer.
  /**
   * \param names The names markers to erase.
   */
  void eraseMarkers(const std::vector<std::string> & names);

  std::map<std::string, InteractiveMarker::SharedPtr> interactive_markers_map_;

  // Properties
  InteractiveMarkerNamespaceProperty * interactive_marker_namespace_property_;
  rviz_common::properties::BoolProperty * show_descriptions_property_;
  rviz_common::properties::BoolProperty * show_axes_property_;
  rviz_common::properties::BoolProperty * show_visual_aids_property_;
  rviz_common::properties::BoolProperty * enable_transparency_property_;

  std::unique_ptr<interactive_markers::InteractiveMarkerClient> interactive_marker_client_;
};  // class InteractiveMarkerDisplay

}  // namespace displays
}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__INTERACTIVE_MARKERS__INTERACTIVE_MARKER_DISPLAY_HPP_
