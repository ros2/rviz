/*
 * Copyright (c) 2011, Willow Garage, Inc.
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

#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__INTERACTIVE_MARKERS__INTERACTIVE_MARKER_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__INTERACTIVE_MARKERS__INTERACTIVE_MARKER_HPP_

#ifndef Q_MOC_RUN
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <OgreVector.h>
#include <OgreQuaternion.h>
#include <OgreSceneNode.h>
#endif

#include <QMenu>

#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_pose.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>

#include <rclcpp/publisher.hpp>

#include "rviz_common/display_context.hpp"
#include "rviz_common/properties/status_property.hpp"
#include "rviz_rendering/objects/axes.hpp"

#include "rviz_default_plugins/displays/interactive_markers/interactive_marker_control.hpp"

#include "rviz_default_plugins/visibility_control.hpp"

namespace rviz_default_plugins
{
namespace displays
{
class RVIZ_DEFAULT_PLUGINS_PUBLIC InteractiveMarker : public QObject
{
  Q_OBJECT

public:
  using SharedPtr = std::shared_ptr<InteractiveMarker>;

  InteractiveMarker(Ogre::SceneNode * scene_node, rviz_common::DisplayContext * context);
  virtual ~InteractiveMarker();

  /// Reset contents to reflect the data from a new message.
  /**
   * \return true if successful, false otherwise.
   */
  bool processMessage(const visualization_msgs::msg::InteractiveMarker & message);

  /// Reset contents to reflect the data from a new message.
  /**
   * \return true if successful, false otherwise.
   */
  void processMessage(const visualization_msgs::msg::InteractiveMarkerPose & message);

  /// Called every frame update.
  void update();

  /// Directly set the pose relative to the parent frame.
  /**
   * If publish is set to true, then the change is published.
   */
  void setPose(
    Ogre::Vector3 position,
    Ogre::Quaternion orientation,
    const std::string & control_name);

  void translate(Ogre::Vector3 delta_position, const std::string & control_name);

  void rotate(Ogre::Quaternion delta_orientation, const std::string & control_name);

  /// Schedule a pose reset once dragging is finished.
  void requestPoseUpdate(Ogre::Vector3 position, Ogre::Quaternion orientation);

  void startDragging();

  void stopDragging();

  inline const Ogre::Vector3 & getPosition()
  {
    return position_;
  }

  inline const Ogre::Quaternion & getOrientation()
  {
    return orientation_;
  }

  inline float getSize()
  {
    return scale_;
  }

  inline const std::string & getReferenceFrame()
  {
    return reference_frame_;
  }

  inline const std::string & getName()
  {
    return name_;
  }

  /// Show name above marker.
  void setShowDescription(bool show);

  /// Show axes in origin.
  void setShowAxes(bool show);

  /// Show visual helpers while dragging.
  void setShowVisualAids(bool show);

  /**
   * \return true if the mouse event was intercepted, false if it was ignored.
   */
  bool handleMouseEvent(rviz_common::ViewportMouseEvent & event, const std::string & control_name);

  /// Supports selection and menu events from a 3D cursor.
  /**
   * \param event A struct holding certain event data (see full description
   *   InteractiveMarkerControl::handle3DCursorEvent).
   * \param cursor_pos The world-relative position of the 3D cursor.
   * \param cursor_rot The world-relative orientation of the 3D cursor.
   * \param control_name The name of the child InteractiveMarkerControl calling this function.
   * \return true if the cursor event was intercepted, false if it was ignored.
   */
  bool handle3DCursorEvent(
    rviz_common::ViewportMouseEvent & event,
    const Ogre::Vector3 & cursor_pos,
    const Ogre::Quaternion & cursor_rot,
    const std::string & control_name);

  /// Pop up the context menu for this marker.
  /**
   * \param event A struct holding certain event data (see full description on
   *   InteractiveMarkerControl::handle3DCursorEvent).
   * \param control_name The name of the InteractiveMarkerControl that was selected.
   * \param three_d_point The world-relative position associated with this mouse-click or cursor event.
   * \param valid_point true if three_d_point is valid (e.g. if the mouse ray was successfully
   *   intersected with marker geometry).
   */
  void showMenu(
    rviz_common::ViewportMouseEvent & event,
    const std::string & control_name,
    const Ogre::Vector3 & three_d_point,
    bool valid_point);

  /// Publish the current marker pose and name.
  void publishFeedback(
    visualization_msgs::msg::InteractiveMarkerFeedback & feedback,
    bool mouse_point_valid = false,
    const Ogre::Vector3 & mouse_point_rel_world = Ogre::Vector3(0, 0, 0));

  inline bool hasMenu()
  {
    return has_menu_;
  }

  /**
   * \return A shared_ptr to the QMenu owned by this InteractiveMarker.
   */
  inline std::shared_ptr<QMenu> getMenu()
  {
    return menu_;
  }

Q_SIGNALS:
  void userFeedback(visualization_msgs::msg::InteractiveMarkerFeedback & feedback);

  void statusUpdate(
    rviz_common::properties::StatusProperty::Level level,
    const std::string & name,
    const std::string & text);

protected Q_SLOTS:
  void handleMenuSelect(int menu_item_id);

private:
  void publishPose();

  /// Update the `controls_` member
  void updateControls(
    const std::vector<visualization_msgs::msg::InteractiveMarkerControl> & controls);

  /// Update the `menu_` member
  void createMenu(const std::vector<visualization_msgs::msg::MenuEntry> & entries);

  /**
   * Recursively append menu and submenu entries to menu, based on a
   * vector of menu entry id numbers describing the menu entries at the
   * current level.
   */
  void populateMenu(QMenu * menu, std::vector<uint32_t> & ids);

  QString makeMenuString(const std::string & entry);

  /// Set the pose of the parent frame, relative to the fixed frame
  void updateReferencePose();

  rviz_common::DisplayContext * context_;

  std::string reference_frame_;

  rclcpp::Time reference_time_;

  bool frame_locked_;

  /// Node representing reference frame in tf, like /map, /base_link, /head, etc.
  Ogre::SceneNode * reference_node_;

  /// Position being controlled, relative to reference frame.
  Ogre::Vector3 position_;

  /// Orientation being controlled, relative to reference frame.
  Ogre::Quaternion orientation_;

  /// Has the pose changed since the last feedback was sent?
  bool pose_changed_;

  typedef std::shared_ptr<InteractiveMarkerControl> InteractiveMarkerControlPtr;
  typedef std::map<std::string, InteractiveMarkerControlPtr> M_ControlPtr;

  M_ControlPtr controls_;

  std::string name_;

  std::string description_;

  bool dragging_;

  bool pose_update_requested_;

  Ogre::Vector3 requested_position_;

  Ogre::Quaternion requested_orientation_;

  float scale_;

  std::shared_ptr<QMenu> menu_;

  bool has_menu_;

  /// Helper to more simply represent the menu tree.
  struct MenuNode
  {
    visualization_msgs::msg::MenuEntry entry;
    std::vector<uint32_t> child_ids;
  };

  /// Maps menu index to menu entry and item.
  std::map<uint32_t, MenuNode> menu_entries_;

  /// Helper to store the top level of the menu tree.
  std::vector<uint32_t> top_level_menu_ids_;

  /// Which control has popped up the menu
  std::string last_control_name_;

  double heart_beat_t_;

  // Visual aids

  std::unique_ptr<rviz_rendering::Axes> axes_;

  InteractiveMarkerControlPtr description_control_;

  std::string topic_ns_;

  std::string client_id_;

  std::recursive_mutex mutex_;

  std::shared_ptr<std::thread> sys_thread_;

  bool got_3d_point_for_menu_;

  Ogre::Vector3 three_d_point_for_menu_;

  bool show_visual_aids_;
};  // class InteractiveMarker

}  // namespace displays
}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__INTERACTIVE_MARKERS__INTERACTIVE_MARKER_HPP_
