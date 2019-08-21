/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * Copyright (c) 2018, Bosch Software Innovations, GmbH.
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

#include "rviz_default_plugins/displays/tf/tf_display.hpp"

#include <algorithm>
#include <cassert>
#include <memory>
#include <set>
#include <string>
#include <vector>
#include <utility>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "rviz_rendering/objects/arrow.hpp"
#include "rviz_rendering/objects/axes.hpp"
#include "rviz_rendering/objects/movable_text.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/frame_manager_iface.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/msg_conversions.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/quaternion_property.hpp"
#include "rviz_common/properties/string_property.hpp"
#include "rviz_common/properties/vector_property.hpp"
#include "rviz_common/interaction/forwards.hpp"
#include "rviz_common/interaction/selection_manager.hpp"
#include "rviz_common/uniform_string_stream.hpp"
#include "rviz_default_plugins/displays/tf/frame_info.hpp"
#include "rviz_default_plugins/displays/tf/frame_selection_handler.hpp"
#include "rviz_default_plugins/transformation/tf_wrapper.hpp"

using rviz_common::interaction::SelectionHandler;
using rviz_common::properties::BoolProperty;
using rviz_common::properties::FloatProperty;
using rviz_common::properties::StatusProperty;
using rviz_common::properties::StringProperty;
using rviz_common::properties::Property;
using rviz_common::properties::QuaternionProperty;
using rviz_common::properties::VectorProperty;
using rviz_common::interaction::Picked;
using rviz_rendering::Axes;
using rviz_rendering::Arrow;
using rviz_rendering::MovableText;

namespace rviz_default_plugins
{

namespace displays
{

TFDisplay::TFDisplay()
: update_timer_(0.0f),
  changing_single_frame_enabled_state_(false),
  transformer_guard_(
    std::make_unique<rviz_default_plugins::transformation::TransformerGuard<
      rviz_default_plugins::transformation::TFFrameTransformer>>(this, "TF"))
{
  show_names_property_ = new BoolProperty(
    "Show Names",
    false,
    "Whether or not names should be shown next to the frames.",
    this,
    SLOT(updateShowNames()));

  show_axes_property_ = new BoolProperty(
    "Show Axes",
    true,
    "Whether or not the axes of each frame should be shown.",
    this,
    SLOT(updateShowAxes()));

  show_arrows_property_ = new BoolProperty(
    "Show Arrows",
    true,
    "Whether or not arrows from child to parent should be shown.",
    this,
    SLOT(updateShowArrows()));

  scale_property_ = new FloatProperty(
    "Marker Scale",
    1,
    "Scaling factor for all names, axes and arrows.",
    this);

  update_rate_property_ = new FloatProperty(
    "Update Interval",
    0,
    "The interval, in seconds, at which to update the frame transforms. "
    "0 means to do so every update cycle.",
    this);
  update_rate_property_->setMin(0);

  frame_timeout_property_ = new FloatProperty(
    "Frame Timeout",
    15,
    "The length of time, in seconds, before a frame that has not been updated is considered"
    " \"dead\".  For 1/3 of this time the frame will appear correct, for the second 1/3rd it will"
    " fade to gray, and then it will fade out completely.",
    this);
  frame_timeout_property_->setMin(1);

  frames_category_ = new Property("Frames", QVariant(), "The list of all frames.", this);

  all_enabled_property_ = new BoolProperty(
    "All Enabled",
    true,
    "Whether all the frames should be enabled or not.",
    frames_category_,
    SLOT(allEnabledChanged()),
    this);

  tree_category_ = new Property(
    "Tree",
    QVariant(),
    "A tree-view of the frames, showing the parent/child relationships.",
    this);
}

TFDisplay::~TFDisplay()
{
  if (initialized()) {
    root_node_->removeAndDestroyAllChildren();
    scene_manager_->destroySceneNode(root_node_);
  }
}

void TFDisplay::onInitialize()
{
  frame_config_enabled_state_.clear();

  root_node_ = scene_node_->createChildSceneNode();

  names_node_ = root_node_->createChildSceneNode();
  arrows_node_ = root_node_->createChildSceneNode();
  axes_node_ = root_node_->createChildSceneNode();

  transformer_guard_->initialize(context_);
}

void TFDisplay::load(const rviz_common::Config & config)
{
  rviz_common::Display::load(config);

  // Load the enabled state for all frames specified in the config, and store
  // the values in a map so that the enabled state can be properly set once
  // the frame is created
  rviz_common::Config c = config.mapGetChild("Frames");
  for (auto iter = c.mapIterator(); iter.isValid(); iter.advance()) {
    QString key = iter.currentKey();
    if (key != "All Enabled") {
      const rviz_common::Config & child = iter.currentChild();
      bool enabled = child.mapGetChild("Value").getValue().toBool();

      frame_config_enabled_state_[key.toStdString()] = enabled;
    }
  }
}

void TFDisplay::clear()
{
  tree_category_->removeChildren();

  // Clear the frames category, except for the "All enabled" property, which is first.
  frames_category_->removeChildren(1);

  S_FrameInfo to_delete;
  for (auto & frame : frames_) {
    to_delete.insert(frame.second);
  }

  for (auto & frame : to_delete) {
    deleteFrame(frame, false);
  }

  frames_.clear();

  update_timer_ = 0.0f;

  clearStatuses();
}

void TFDisplay::onEnable()
{
  root_node_->setVisible(true);

  names_node_->setVisible(show_names_property_->getBool());
  arrows_node_->setVisible(show_arrows_property_->getBool());
  axes_node_->setVisible(show_axes_property_->getBool());
}

void TFDisplay::onDisable()
{
  root_node_->setVisible(false);
  clear();
}

void TFDisplay::updateShowNames()
{
  names_node_->setVisible(show_names_property_->getBool());

  for (auto & frame : frames_) {
    frame.second->updateVisibilityFromFrame();
  }
}

void TFDisplay::updateShowAxes()
{
  axes_node_->setVisible(show_axes_property_->getBool());

  for (auto & frame : frames_) {
    frame.second->updateVisibilityFromFrame();
  }
}

void TFDisplay::updateShowArrows()
{
  arrows_node_->setVisible(show_arrows_property_->getBool());

  for (auto & frame : frames_) {
    frame.second->updateVisibilityFromFrame();
  }
}

void TFDisplay::allEnabledChanged()
{
  if (changing_single_frame_enabled_state_) {
    return;
  }
  bool enabled = all_enabled_property_->getBool();

  for (auto & frame : frames_) {
    frame.second->enabled_property_->setBool(enabled);
  }
}

void TFDisplay::update(float wall_dt, float ros_dt)
{
  if (!transformer_guard_->checkTransformer()) {
    return;
  }

  (void) ros_dt;
  update_timer_ += wall_dt;
  float update_rate = update_rate_property_->getFloat();
  if (update_rate < 0.0001f || update_timer_ > update_rate * 1000000000) {
    updateFrames();

    update_timer_ = 0.0f;
  }
}

void TFDisplay::updateFrames()
{
  typedef std::vector<std::string> V_string;
  V_string frames = context_->getFrameManager()->getAllFrameNames();
  std::sort(frames.begin(), frames.end());

  S_FrameInfo current_frames = createOrUpdateFrames(frames);
  deleteObsoleteFrames(current_frames);

  context_->queueRender();
}

S_FrameInfo TFDisplay::createOrUpdateFrames(const std::vector<std::string> & frames)
{
  S_FrameInfo current_frames;
  for (auto & frame : frames) {
    if (frame.empty()) {
      continue;
    }

    FrameInfo * info = getFrameInfo(frame);
    if (!info) {
      info = createFrame(frame);
    } else {
      updateFrame(info);
    }

    current_frames.insert(info);
  }
  return current_frames;
}

FrameInfo * TFDisplay::getFrameInfo(const std::string & frame)
{
  auto it = frames_.find(frame);
  if (it == frames_.end()) {
    return nullptr;
  }

  return it->second;
}

void TFDisplay::deleteObsoleteFrames(S_FrameInfo & current_frames)
{
  S_FrameInfo to_delete;
  for (auto & frame : frames_) {
    if (current_frames.find(frame.second) == current_frames.end()) {
      to_delete.insert(frame.second);
    }
  }

  for (auto & frame : to_delete) {
    deleteFrame(frame, true);
  }
}

FrameInfo * TFDisplay::createFrame(const std::string & frame)
{
  auto info = new FrameInfo(this);
  frames_.insert(std::make_pair(frame, info));

  info->name_ = frame;
  info->last_update_ = tf2::get_now();
  info->axes_ = new Axes(scene_manager_, axes_node_, 0.2f, 0.02f);
  info->axes_->getSceneNode()->setVisible(show_axes_property_->getBool());
  info->selection_handler_ =
    rviz_common::interaction::createSelectionHandler<FrameSelectionHandler>(info, this, context_);
  info->selection_handler_->addTrackedObjects(info->axes_->getSceneNode());

  info->name_text_ = new MovableText(frame, "Liberation Sans", 0.1f);
  info->name_text_->setTextAlignment(MovableText::H_CENTER, MovableText::V_BELOW);
  info->name_node_ = names_node_->createChildSceneNode();
  info->name_node_->attachObject(info->name_text_);
  info->name_node_->setVisible(show_names_property_->getBool());

  info->parent_arrow_ = new Arrow(scene_manager_, arrows_node_, 1.0f, 0.01f, 1.0f, 0.08f);
  info->parent_arrow_->getSceneNode()->setVisible(false);
  info->parent_arrow_->setHeadColor(FrameInfo::ARROW_HEAD_COLOR);
  info->parent_arrow_->setShaftColor(FrameInfo::ARROW_SHAFT_COLOR);

  info->enabled_property_ = new BoolProperty(
    QString::fromStdString(info->name_),
    true,
    "Enable or disable this individual frame.",
    frames_category_,
    SLOT(updateVisibilityFromFrame()),
    info);

  info->parent_property_ = new StringProperty(
    "Parent", "",
    "Parent of this frame.  (Not editable)",
    info->enabled_property_);
  info->parent_property_->setReadOnly(true);

  info->position_property_ = new VectorProperty(
    "Position", Ogre::Vector3::ZERO,
    "Position of this frame, in the current Fixed Frame.  (Not editable)",
    info->enabled_property_);
  info->position_property_->setReadOnly(true);

  info->orientation_property_ = new QuaternionProperty(
    "Orientation", Ogre::Quaternion::IDENTITY,
    "Orientation of this frame, in the current Fixed Frame.  (Not editable)",
    info->enabled_property_);
  info->orientation_property_->setReadOnly(true);

  info->rel_position_property_ = new VectorProperty(
    "Relative Position", Ogre::Vector3::ZERO,
    "Position of this frame, relative to it's parent frame.  (Not editable)",
    info->enabled_property_);
  info->rel_position_property_->setReadOnly(true);

  info->rel_orientation_property_ = new QuaternionProperty(
    "Relative Orientation",
    Ogre::Quaternion::IDENTITY,
    "Orientation of this frame, relative to it's parent frame.  (Not editable)",
    info->enabled_property_);
  info->rel_orientation_property_->setReadOnly(true);

  // If the current frame was specified as disabled in the config file
  // then its enabled state must be updated accordingly
  if (frame_config_enabled_state_.count(frame) > 0 && !frame_config_enabled_state_[frame]) {
    info->enabled_property_->setBool(false);
  }

  updateFrame(info);

  return info;
}

void TFDisplay::updateFrame(FrameInfo * frame)
{
  auto tf_wrapper = std::dynamic_pointer_cast<transformation::TFWrapper>(
    context_->getFrameManager()->getConnector().lock());

  if (tf_wrapper) {
    std::shared_ptr<tf2::BufferCore> tf_buffer = tf_wrapper->getBuffer();

    // Check last received time so we can grey out/fade out frames that have stopped being published
    tf2::TimePoint latest_time;

    std::string stripped_fixed_frame = fixed_frame_.toStdString();
    if (stripped_fixed_frame[0] == '/') {
      stripped_fixed_frame = stripped_fixed_frame.substr(1);
    }
    try {
      tf_buffer->_getLatestCommonTime(
        tf_buffer->_validateFrameId("get_latest_common_time", stripped_fixed_frame),
        tf_buffer->_validateFrameId("get_latest_common_time", frame->name_),
        latest_time,
        nullptr);
    } catch (const tf2::LookupException & e) {
      logTransformationException(stripped_fixed_frame, frame->name_, e.what());
      return;
    }

    frame->setLastUpdate(latest_time);

    double age = tf2::durationToSec(tf2::get_now() - frame->last_update_);
    double frame_timeout = frame_timeout_property_->getFloat();
    if (age > frame_timeout) {
      frame->setVisible(false);
      return;
    }
    frame->updateColorForAge(age, frame_timeout);

    setStatusStd(StatusProperty::Ok, frame->name_, "Transform OK");

    Ogre::Vector3 position(0, 0, 0);
    Ogre::Quaternion orientation(1.0, 0.0, 0.0, 0.0);
    if (!context_->getFrameManager()->getTransform(frame->name_, position, orientation)) {
      rviz_common::UniformStringStream ss;
      ss << "No transform from [" << frame->name_ << "] to [" << fixed_frame_.toStdString() << "]";
      setStatusStd(StatusProperty::Warn, frame->name_, ss.str());
      frame->setVisible(false);
      return;
    }

    frame->updatePositionAndOrientation(position, orientation, scale_property_->getFloat());
    frame->setNamesVisible(show_names_property_->getBool());
    frame->setAxesVisible(show_axes_property_->getBool());

    std::string old_parent = frame->parent_;
    frame->parent_.clear();
    bool has_parent = tf_buffer->_getParent(frame->name_, tf2::TimePointZero, frame->parent_);
    if (has_parent) {
      if (hasNoTreePropertyOrParentChanged(frame, old_parent)) {
        updateParentTreeProperty(frame);
      }

      updateRelativePositionAndOrientation(frame, tf_buffer);

      if (show_arrows_property_->getBool()) {
        updateParentArrowIfTransformExists(frame, position);
      } else {
        frame->setParentArrowVisible(false);
      }
    } else {
      if (hasNoTreePropertyOrParentChanged(frame, old_parent)) {
        frame->updateTreeProperty(tree_category_);
      }

      frame->setParentArrowVisible(false);
    }

    frame->parent_property_->setStdString(frame->parent_);
    frame->selection_handler_->setParentName(frame->parent_);
  }
}

void TFDisplay::updateParentTreeProperty(FrameInfo * frame) const
{
  // Look up the new parent.
  auto parent_it = frames_.find(frame->parent_);
  if (parent_it != frames_.end()) {
    FrameInfo * parent = parent_it->second;

    // If the parent has a tree property, make a new tree property for this frame.
    if (parent->tree_property_) {
      frame->updateTreeProperty(parent->tree_property_);
    }
  }
}

/// If this frame has no tree property or the parent has changed,
bool TFDisplay::hasNoTreePropertyOrParentChanged(
  const FrameInfo * frame,
  const std::string & old_parent) const
{
  return !frame->tree_property_ || old_parent != frame->parent_;
}

void TFDisplay::logTransformationException(
  const std::string & parent_frame,
  const std::string & child_frame,
  const std::string & message) const
{
  RVIZ_COMMON_LOG_DEBUG_STREAM(
    "Error transforming from frame '" << parent_frame.c_str() <<
      "' to frame '" << child_frame.c_str() <<
      "' with fixed frame '" << qPrintable(fixed_frame_) << "': " << message);
}

/// set the position/orientation relative to the parent frame
void TFDisplay::updateRelativePositionAndOrientation(
  const FrameInfo * frame,
  std::shared_ptr<tf2::BufferCore> tf_buffer) const
{
  geometry_msgs::msg::TransformStamped transform;
  transform.transform.translation = geometry_msgs::msg::Vector3();
  transform.transform.rotation = geometry_msgs::msg::Quaternion();

  try {
    transform = tf_buffer->lookupTransform(frame->parent_, frame->name_, tf2::TimePointZero);
  } catch (const tf2::LookupException & e) {
    logTransformationException(frame->parent_, frame->name_, e.what());
  } catch (const tf2::TransformException & e) {
    logTransformationException(frame->parent_, frame->name_, e.what());
  }

  frame->rel_position_property_->setVector(
    rviz_common::vector3MsgToOgre(transform.transform.translation));
  frame->rel_orientation_property_->setQuaternion(
    rviz_common::quaternionMsgToOgre(transform.transform.rotation));
}

void TFDisplay::updateParentArrowIfTransformExists(
  FrameInfo * frame,
  const Ogre::Vector3 & position) const
{
  Ogre::Vector3 parent_position(0, 0, 0);
  Ogre::Quaternion parent_orientation(1.0f, 0.0f, 0.0f, 0.0f);
  if (!context_->getFrameManager()->getTransform(
      frame->parent_, parent_position, parent_orientation))
  {
    logTransformationException(frame->parent_, frame->name_);
  } else {
    frame->setParentArrowVisible(show_arrows_property_->getBool());
    frame->updateParentArrow(position, parent_position, scale_property_->getFloat());
  }
}


void TFDisplay::deleteFrame(FrameInfo * frame, bool delete_properties)
{
  auto it = frames_.find(frame->name_);
  assert(it != frames_.end());

  frames_.erase(it);

  delete frame->axes_;
  context_->getHandlerManager()->removeHandler(frame->axes_coll_);
  delete frame->parent_arrow_;
  delete frame->name_text_;
  scene_manager_->destroySceneNode(frame->name_node_);
  if (delete_properties) {
    delete frame->enabled_property_;
    delete frame->tree_property_;
  }
  delete frame;
}

void TFDisplay::fixedFrameChanged()
{
  update_timer_ = update_rate_property_->getFloat();
}

void TFDisplay::reset()
{
  rviz_common::Display::reset();
  clear();
}

}  // namespace displays
}  // namespace rviz_default_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::TFDisplay, rviz_common::Display)
