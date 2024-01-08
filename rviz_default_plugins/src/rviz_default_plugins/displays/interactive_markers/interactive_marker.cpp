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

#include <QMenu>

#include <map>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreMaterialManager.h>
#include <OgreResourceGroupManager.h>
#include <OgreSubEntity.h>
#include <OgreMath.h>
#include <OgreRenderWindow.h>

#include "interactive_markers/tools.hpp"

#include "rclcpp/rclcpp.hpp"

#include "rviz_common/display_context.hpp"
#include "rviz_common/interaction/view_picker_iface.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_rendering/geometry.hpp"
#include "rviz_rendering/render_window.hpp"

#include "rviz_default_plugins/displays/interactive_markers/integer_action.hpp"
#include "rviz_default_plugins/displays/interactive_markers/interactive_marker.hpp"

namespace rviz_default_plugins
{
namespace displays
{

InteractiveMarker::InteractiveMarker(
  Ogre::SceneNode * scene_node, rviz_common::DisplayContext * context)
: context_(context),
  pose_changed_(false),
  dragging_(false),
  pose_update_requested_(false),
  heart_beat_t_(0),
  show_visual_aids_(false)
{
  reference_node_ = scene_node->createChildSceneNode();
  axes_ = std::make_unique<rviz_rendering::Axes>(
    context->getSceneManager(), reference_node_, 1.0f, 0.05f);
}

InteractiveMarker::~InteractiveMarker()
{
  context_->getSceneManager()->destroySceneNode(reference_node_);
}

void InteractiveMarker::processMessage(
  const visualization_msgs::msg::InteractiveMarkerPose & message)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  Ogre::Vector3 position(message.pose.position.x, message.pose.position.y, message.pose.position.z);
  Ogre::Quaternion orientation(
    message.pose.orientation.w,
    message.pose.orientation.x,
    message.pose.orientation.y,
    message.pose.orientation.z);

  if (orientation.w == 0 && orientation.x == 0 && orientation.y == 0 && orientation.z == 0) {
    orientation.w = 1.0;
  }

  reference_time_ = rclcpp::Time(message.header.stamp, RCL_ROS_TIME);
  reference_frame_ = message.header.frame_id;
  frame_locked_ = (message.header.stamp == builtin_interfaces::msg::Time());

  requestPoseUpdate(position, orientation);
  context_->queueRender();
}

bool InteractiveMarker::processMessage(const visualization_msgs::msg::InteractiveMarker & message)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  // copy values
  name_ = message.name;
  description_ = message.description;

  if (message.controls.size() == 0) {
    Q_EMIT statusUpdate(rviz_common::properties::StatusProperty::Ok, name_, "Marker empty.");
    return false;
  }

  scale_ = message.scale;

  reference_time_ = rclcpp::Time(message.header.stamp, RCL_ROS_TIME);
  reference_frame_ = message.header.frame_id;
  frame_locked_ = (message.header.stamp == builtin_interfaces::msg::Time());

  position_ = Ogre::Vector3(
    message.pose.position.x, message.pose.position.y, message.pose.position.z);

  orientation_ = Ogre::Quaternion(
    message.pose.orientation.w,
    message.pose.orientation.x,
    message.pose.orientation.y,
    message.pose.orientation.z);

  pose_changed_ = false;

  // setup axes
  axes_->setPosition(position_);
  axes_->setOrientation(orientation_);
  axes_->set(scale_, scale_ * 0.05);

  has_menu_ = message.menu_entries.size() > 0;

  updateReferencePose();

  updateControls(message.controls);

  description_control_ = std::make_shared<InteractiveMarkerControl>(
    context_, reference_node_, this);
  description_control_->processMessage(interactive_markers::makeTitle(message));

  // Create menu
  menu_entries_.clear();
  menu_.reset();
  if (has_menu_) {
    createMenu(message.menu_entries);
  }

  if (frame_locked_) {
    std::ostringstream s;
    s << "Locked to frame " << reference_frame_;
    Q_EMIT statusUpdate(rviz_common::properties::StatusProperty::Ok, name_, s.str());
  } else {
    Q_EMIT statusUpdate(rviz_common::properties::StatusProperty::Ok, name_, "Position is fixed.");
  }
  return true;
}

void InteractiveMarker::update()
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (frame_locked_) {
    updateReferencePose();
  }

  for (auto & control : controls_) {
    control.second->update();
  }
  if (description_control_) {
    description_control_->update();
  }

  if (dragging_) {
    if (pose_changed_) {
      publishPose();
    }
  }
}

void InteractiveMarker::setPose(
  Ogre::Vector3 position, Ogre::Quaternion orientation, const std::string & control_name)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  position_ = position;
  orientation_ = orientation;
  pose_changed_ = true;
  last_control_name_ = control_name;

  axes_->setPosition(position_);
  axes_->setOrientation(orientation_);

  for (auto & control : controls_) {
    control.second->interactiveMarkerPoseChanged(position_, orientation_);
  }
  if (description_control_) {
    description_control_->interactiveMarkerPoseChanged(position_, orientation_);
  }
}

void InteractiveMarker::translate(Ogre::Vector3 delta_position, const std::string & control_name)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  setPose(position_ + delta_position, orientation_, control_name);
}

void InteractiveMarker::rotate(
  Ogre::Quaternion delta_orientation, const std::string & control_name)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  setPose(position_, delta_orientation * orientation_, control_name);
}

void InteractiveMarker::requestPoseUpdate(Ogre::Vector3 position, Ogre::Quaternion orientation)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (dragging_) {
    pose_update_requested_ = true;
    requested_position_ = position;
    requested_orientation_ = orientation;
  } else {
    updateReferencePose();
    setPose(position, orientation, "");
  }
}

void InteractiveMarker::startDragging()
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  dragging_ = true;
  pose_changed_ = false;
}

void InteractiveMarker::stopDragging()
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  dragging_ = false;
  if (pose_update_requested_) {
    updateReferencePose();
    setPose(requested_position_, requested_orientation_, "");
    pose_update_requested_ = false;
  }
}

void InteractiveMarker::setShowDescription(bool show)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (description_control_.get()) {
    description_control_->setVisible(show);
  }
}

void InteractiveMarker::setShowAxes(bool show)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  axes_->getSceneNode()->setVisible(show);
}

void InteractiveMarker::setShowVisualAids(bool show)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  for (auto & control : controls_) {
    control.second->setShowVisualAids(show);
  }
  show_visual_aids_ = show;
}

bool InteractiveMarker::handleMouseEvent(
  rviz_common::ViewportMouseEvent & event, const std::string & control_name)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  if (event.acting_button == Qt::LeftButton) {
    Ogre::Vector3 point_rel_world;
    bool got_3D_point = context_->getViewPicker()->get3DPoint(
      event.panel, event.x, event.y, point_rel_world);

    visualization_msgs::msg::InteractiveMarkerFeedback feedback;
    feedback.control_name = control_name;
    feedback.marker_name = name_;

    // make sure we've published a last pose update
    feedback.event_type = static_cast<uint8_t>(
      visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE);
    publishFeedback(feedback, got_3D_point, point_rel_world);

    feedback.event_type = (event.type == QEvent::MouseButtonPress ?
      static_cast<uint8_t>(visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_DOWN) :
      static_cast<uint8_t>(visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP));
    publishFeedback(feedback, got_3D_point, point_rel_world);
  }

  if (!dragging_ && menu_.get()) {
    // Event.right() will be false during a right-button-up event.  We
    // want to swallow (with the "return true") all other
    // right-button-related mouse events.
    if (event.right()) {
      return true;
    }
    if (event.rightUp() && event.buttons_down == Qt::NoButton) {
      // Save the 3D mouse point to send with the menu feedback, if any.
      Ogre::Vector3 three_d_point;
      bool valid_point = context_->getViewPicker()->get3DPoint(
        event.panel, event.x, event.y, three_d_point);
      showMenu(event, control_name, three_d_point, valid_point);
      return true;
    }
  }

  return false;
}

bool InteractiveMarker::handle3DCursorEvent(
  rviz_common::ViewportMouseEvent & event,
  const Ogre::Vector3 & cursor_pos,
  const Ogre::Quaternion & cursor_rot,
  const std::string & control_name)
{
  (void)cursor_rot;

  std::lock_guard<std::recursive_mutex> lock(mutex_);

  if (event.acting_button == Qt::LeftButton) {
    Ogre::Vector3 point_rel_world = cursor_pos;
    bool got_3D_point = true;

    visualization_msgs::msg::InteractiveMarkerFeedback feedback;
    feedback.control_name = control_name;
    feedback.marker_name = name_;

    // make sure we've published a last pose update
    feedback.event_type = static_cast<uint8_t>(
      visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE);
    publishFeedback(feedback, got_3D_point, point_rel_world);

    feedback.event_type = (event.type == QEvent::MouseButtonPress ?
      static_cast<uint8_t>(visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_DOWN) :
      static_cast<uint8_t>(visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP));
    publishFeedback(feedback, got_3D_point, point_rel_world);
  }

  if (!dragging_ && menu_.get()) {
    // Event.right() will be false during a right-button-up event.  We
    // want to swallow (with the "return true") all other
    // right-button-related mouse events.
    if (event.right()) {
      return true;
    }
    if (event.rightUp() && event.buttons_down == Qt::NoButton) {
      // Save the 3D mouse point to send with the menu feedback, if any.
      Ogre::Vector3 three_d_point = cursor_pos;
      bool valid_point = true;
      Ogre::Vector2 mouse_pos = rviz_rendering::project3DPointToViewportXY(
        rviz_rendering::RenderWindowOgreAdapter::getOgreViewport(event.panel->getRenderWindow()),
        cursor_pos);
      QCursor::setPos(event.panel->mapToGlobal(QPoint(mouse_pos.x, mouse_pos.y)));
      showMenu(event, control_name, three_d_point, valid_point);
      return true;
    }
  }

  return false;
}

void InteractiveMarker::showMenu(
  rviz_common::ViewportMouseEvent & event,
  const std::string & control_name,
  const Ogre::Vector3 & three_d_point,
  bool valid_point)
{
  // Save the 3D mouse point to send with the menu feedback, if any.
  got_3d_point_for_menu_ = valid_point;
  three_d_point_for_menu_ = three_d_point;

  event.panel->showContextMenu(menu_);
  last_control_name_ = control_name;
}

void InteractiveMarker::publishFeedback(
  visualization_msgs::msg::InteractiveMarkerFeedback & feedback,
  bool mouse_point_valid,
  const Ogre::Vector3 & mouse_point_rel_world)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  feedback.marker_name = name_;

  if (frame_locked_) {
    // frame-locked IMs will return their pose in the same coordinate frame
    // as they were set up with (the "reference frame").
    // The transformation's timestamp will be the one used for placing
    // the reference frame into the fixed frame
    feedback.header.frame_id = reference_frame_;
    feedback.header.stamp = reference_time_;
    feedback.pose.position.x = position_.x;
    feedback.pose.position.y = position_.y;
    feedback.pose.position.z = position_.z;
    feedback.pose.orientation.x = orientation_.x;
    feedback.pose.orientation.y = orientation_.y;
    feedback.pose.orientation.z = orientation_.z;
    feedback.pose.orientation.w = orientation_.w;

    feedback.mouse_point_valid = mouse_point_valid;
    if (mouse_point_valid) {
      Ogre::Vector3 mouse_rel_reference = reference_node_->convertWorldToLocalPosition(
        mouse_point_rel_world);
      feedback.mouse_point.x = mouse_rel_reference.x;
      feedback.mouse_point.y = mouse_rel_reference.y;
      feedback.mouse_point.z = mouse_rel_reference.z;
    }
  } else {
    // Timestamped IMs will return feedback in RViz's fixed frame
    feedback.header.frame_id = context_->getFixedFrame().toStdString();
    // This should be rclcpp::Time::now(), but then the computer running
    // RViz has to be time-synced with the server
    feedback.header.stamp = rclcpp::Time();

    Ogre::Vector3 world_position = reference_node_->convertLocalToWorldPosition(position_);
    Ogre::Quaternion world_orientation = reference_node_->convertLocalToWorldOrientation(
      orientation_);

    feedback.pose.position.x = world_position.x;
    feedback.pose.position.y = world_position.y;
    feedback.pose.position.z = world_position.z;
    feedback.pose.orientation.x = world_orientation.x;
    feedback.pose.orientation.y = world_orientation.y;
    feedback.pose.orientation.z = world_orientation.z;
    feedback.pose.orientation.w = world_orientation.w;

    feedback.mouse_point_valid = mouse_point_valid;
    feedback.mouse_point.x = mouse_point_rel_world.x;
    feedback.mouse_point.y = mouse_point_rel_world.y;
    feedback.mouse_point.z = mouse_point_rel_world.z;
  }

  Q_EMIT userFeedback(feedback);
}

void InteractiveMarker::handleMenuSelect(int menu_item_id)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  std::map<uint32_t, MenuNode>::iterator it = menu_entries_.find(menu_item_id);

  if (it != menu_entries_.end()) {
    visualization_msgs::msg::MenuEntry & entry = it->second.entry;

    std::string command = entry.command;
    uint8_t command_type = entry.command_type;

    if (command_type == visualization_msgs::msg::MenuEntry::FEEDBACK) {
      visualization_msgs::msg::InteractiveMarkerFeedback feedback;
      feedback.event_type = visualization_msgs::msg::InteractiveMarkerFeedback::MENU_SELECT;
      feedback.menu_entry_id = entry.id;
      feedback.control_name = last_control_name_;
      publishFeedback(feedback, got_3d_point_for_menu_, three_d_point_for_menu_);
    } else if (command_type == visualization_msgs::msg::MenuEntry::ROSRUN) {
      std::string sys_cmd = "ros2 run " + command;
      RVIZ_COMMON_LOG_INFO_STREAM("Running system command: " << sys_cmd);
      sys_thread_ = std::shared_ptr<std::thread>(
        new std::thread(std::bind(&system, sys_cmd.c_str())));
    } else if (command_type == visualization_msgs::msg::MenuEntry::ROSLAUNCH) {
      std::string sys_cmd = "ros2 launch " + command;
      RVIZ_COMMON_LOG_INFO_STREAM("Running system command: " << sys_cmd);
      sys_thread_ = std::shared_ptr<std::thread>(
        new std::thread(std::bind(&system, sys_cmd.c_str())));
    }
  }
}

void InteractiveMarker::publishPose()
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  visualization_msgs::msg::InteractiveMarkerFeedback feedback;
  feedback.event_type = visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE;
  feedback.control_name = last_control_name_;
  publishFeedback(feedback);
  pose_changed_ = false;
}

void InteractiveMarker::updateControls(
  const std::vector<visualization_msgs::msg::InteractiveMarkerControl> & controls)
{
  // Instead of just erasing all the old controls and making new ones,
  // we want to preserve as much as possible from the old ones
  // so that we don't lose the drag action in progress if a control is
  // being dragged when this update comes in.

  // Maintain a set of old controls to delete
  std::set<std::string> old_names_to_delete;
  for (const auto & name_control_pair : controls_) {
    old_names_to_delete.insert(name_control_pair.first);
  }

  // Loop over new controls
  // If the name of the control already exists, just call the controls processMessage()
  // function to update it.
  // Otherwise, create a new control.
  for (const auto & control_message : controls) {
    auto search_iter = controls_.find(control_message.name);
    InteractiveMarkerControl::SharedPtr control;

    if (search_iter != controls_.end()) {
      // Use existing control
      control = (*search_iter).second;
    } else {
      // Else make new control
      control = std::make_shared<InteractiveMarkerControl>(context_, reference_node_, this);
      controls_[control_message.name] = control;
    }
    // Update the control with the message data
    control->processMessage(control_message);
    control->setShowVisualAids(show_visual_aids_);

    // Remove control name from the list of those to delete
    old_names_to_delete.erase(control_message.name);
  }

  // Remove old nodes (not present in update message)
  for (const std::string & name : old_names_to_delete) {
    controls_.erase(name);
  }
}

void InteractiveMarker::createMenu(const std::vector<visualization_msgs::msg::MenuEntry> & entries)
{
  menu_.reset(new QMenu());
  top_level_menu_ids_.clear();

  // Put all menu entries into the menu_entries_ map and create the
  // tree of menu entry ids.
  for (const auto & entry : entries) {
    MenuNode node;
    node.entry = entry;
    menu_entries_[entry.id] = node;
    if (entry.parent_id == 0) {
      top_level_menu_ids_.push_back(entry.id);
    } else {
      // Find the parent node and add this entry to the parent's list of children.
      std::map<uint32_t, MenuNode>::iterator parent_it = menu_entries_.find(entry.parent_id);
      if (parent_it == menu_entries_.end()) {
        RVIZ_COMMON_LOG_ERROR_STREAM(
          "interactive marker menu entry " << entry.id << " found before its parent id " <<
            entry.parent_id << ". Ignoring.");
      } else {
        (*parent_it).second.child_ids.push_back(entry.id);
      }
    }
  }
  populateMenu(menu_.get(), top_level_menu_ids_);
}

void InteractiveMarker::populateMenu(QMenu * menu, std::vector<uint32_t> & ids)
{
  for (const auto & id : ids) {
    std::map<uint32_t, MenuNode>::iterator node_it = menu_entries_.find(id);

    assert(node_it != menu_entries_.end());

    MenuNode node = (*node_it).second;

    if (node.child_ids.empty()) {
      IntegerAction * action = new IntegerAction(
        makeMenuString(node.entry.title),
        menu,
        static_cast<int>(node.entry.id));
      connect(action, SIGNAL(triggered(int)), this, SLOT(handleMenuSelect(int)));
      menu->addAction(action);
    } else {
      // make sub-menu
      QMenu * sub_menu = menu->addMenu(makeMenuString(node.entry.title));
      populateMenu(sub_menu, node.child_ids);
    }
  }
}

QString InteractiveMarker::makeMenuString(const std::string & entry)
{
  QString menu_entry;
  if (entry.find("[x]") == 0) {
    menu_entry = QChar(0x2611) + QString::fromStdString(entry.substr(3));
  } else if (entry.find("[ ]") == 0) {
    menu_entry = QChar(0x2610) + QString::fromStdString(entry.substr(3));
  } else {
    menu_entry = QChar(0x3000) + QString::fromStdString(entry);
  }
  return menu_entry;
}

void InteractiveMarker::updateReferencePose()
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  Ogre::Vector3 reference_position;
  Ogre::Quaternion reference_orientation;

  // if we're frame-locked, we need to find out what the most recent transformation time
  // actually is so we send back correct feedback
  if (frame_locked_) {
    const std::string fixed_frame = context_->getFrameManager()->getFixedFrame();
    if (reference_frame_ == fixed_frame) {
      // if the two frames are identical, we don't need to do anything.
      // This should be rclcpp::Time::now(), but then the computer running
      // RViz has to be time-synced with the server
      reference_time_ = rclcpp::Time();
    } else {
      // To get latest common time, call lookupTransform with time=0 and use the stamp on the
      // resultant transform.
      try {
        geometry_msgs::msg::TransformStamped transform =
          context_->getFrameManager()->getTransformer()->lookupTransform(
          reference_frame_, fixed_frame, tf2::TimePoint());
        reference_time_ = rclcpp::Time(transform.header.stamp, RCL_ROS_TIME);
      } catch (...) {
        std::ostringstream oss;
        oss << "Error getting time of latest transform between " << reference_frame_ <<
          " and " << fixed_frame;
        Q_EMIT statusUpdate(rviz_common::properties::StatusProperty::Error, name_, oss.str());
        reference_node_->setVisible(false);
        return;
      }
    }
  }

  if (!context_->getFrameManager()->getTransform(
      reference_frame_,
      rclcpp::Time(0, 0u, context_->getClock()->get_clock_type()),
      reference_position,
      reference_orientation))
  {
    std::string error;
    context_->getFrameManager()->transformHasProblems(
      reference_frame_, rclcpp::Time(), error);
    Q_EMIT statusUpdate(rviz_common::properties::StatusProperty::Error, name_, error);
    reference_node_->setVisible(false);
    return;
  }

  reference_node_->setPosition(reference_position);
  reference_node_->setOrientation(reference_orientation);
  reference_node_->setVisible(true, false);

  context_->queueRender();
}

}  // namespace displays
}  // namespace rviz_default_plugins
