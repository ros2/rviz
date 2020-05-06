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

#include "rviz_default_plugins/displays/tf/frame_info.hpp"

#include <algorithm>

#include <OgreSceneNode.h>

#include "rviz_rendering/objects/arrow.hpp"
#include "rviz_rendering/objects/axes.hpp"
#include "rviz_rendering/objects/movable_text.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/properties/vector_property.hpp"
#include "rviz_common/properties/quaternion_property.hpp"
#include "rviz_default_plugins/displays/tf/frame_selection_handler.hpp"

namespace rviz_default_plugins
{
namespace displays
{

FrameInfo::FrameInfo(TFDisplay * display)
: display_(display),
  axes_(nullptr),
  axes_coll_(0),
  parent_arrow_(nullptr),
  name_text_(nullptr),
  distance_to_parent_(0.0f),
  arrow_orientation_(Ogre::Quaternion::IDENTITY),
  tree_property_(nullptr)
{}

const Ogre::ColourValue FrameInfo::ARROW_HEAD_COLOR(1.0f, 0.1f, 0.6f, 1.0f);
const Ogre::ColourValue FrameInfo::ARROW_SHAFT_COLOR(0.8f, 0.8f, 0.3f, 1.0f);


void FrameInfo::updateVisibilityFromFrame()
{
  bool enabled = enabled_property_->getBool();
  selection_handler_->setEnabled(enabled);
  setEnabled(enabled);
}

void FrameInfo::updateVisibilityFromSelection()
{
  bool enabled = selection_handler_->getEnabled();
  enabled_property_->setBool(enabled);
  setEnabled(enabled);
}

void FrameInfo::setEnabled(bool enabled)
{
  if (name_node_) {
    setNamesVisible(display_->show_names_property_->getBool());
  }

  if (axes_) {
    setAxesVisible(display_->show_axes_property_->getBool());
  }

  if (parent_arrow_) {
    setParentArrowVisible(display_->show_arrows_property_->getBool());
  }

  if (display_->all_enabled_property_->getBool() && !enabled) {
    display_->changing_single_frame_enabled_state_ = true;
    display_->all_enabled_property_->setBool(false);
    display_->changing_single_frame_enabled_state_ = false;
  }

  // Update the configuration that stores the enabled state of all frames
  display_->frame_config_enabled_state_[this->name_] = enabled;

  display_->context_->queueRender();
}

void FrameInfo::updatePositionAndOrientation(
  const Ogre::Vector3 & position, const Ogre::Quaternion & orientation, float scale)
{
  selection_handler_->setPosition(position);
  selection_handler_->setOrientation(orientation);
  axes_->setPosition(position);
  axes_->setOrientation(orientation);
  axes_->setScale(Ogre::Vector3(scale, scale, scale));

  name_node_->setPosition(position);
  name_node_->setScale(scale, scale, scale);

  position_property_->setVector(position);
  orientation_property_->setQuaternion(orientation);
}

void FrameInfo::updateTreeProperty(rviz_common::properties::Property * parent)
{
  if (!tree_property_) {
    tree_property_ = new rviz_common::properties::Property(
      QString::fromStdString(name_), QVariant(), "", parent);
  } else {
    tree_property_->setParent(parent);
    tree_property_->setName(QString::fromStdString(name_));
    tree_property_->setValue(QVariant());
    tree_property_->setDescription("");
  }
}

void FrameInfo::setVisible(bool show_frame)
{
  setNamesVisible(show_frame);
  setAxesVisible(show_frame);
  setParentArrowVisible(show_frame);
}

void FrameInfo::setNamesVisible(bool show_names)
{
  bool frame_enabled = enabled_property_->getBool();
  name_node_->setVisible(show_names && frame_enabled);
}

void FrameInfo::setAxesVisible(bool show_axes)
{
  bool frame_enabled = enabled_property_->getBool();
  axes_->getSceneNode()->setVisible(show_axes && frame_enabled);
}

void FrameInfo::setParentArrowVisible(bool show_parent_arrow)
{
  bool frame_enabled = enabled_property_->getBool();
  if (distance_to_parent_ > 0.001f) {
    parent_arrow_->getSceneNode()->setVisible(show_parent_arrow && frame_enabled);
  } else {
    parent_arrow_->getSceneNode()->setVisible(false);
  }
}

void FrameInfo::setLastUpdate(const tf2::TimePoint & latest_time)
{
  if ((latest_time != last_time_to_fixed_) || (latest_time == tf2::TimePointZero)) {
    last_update_ = tf2::get_now();
    last_time_to_fixed_ = latest_time;
  }
}

Ogre::ColourValue lerpColor(const Ogre::ColourValue & start, const Ogre::ColourValue & end, float t)
{
  return start * t + end * (1 - t);
}

/// Fade from color -> grey, then grey -> fully transparent
void FrameInfo::updateColorForAge(double age, double frame_timeout) const
{
  double one_third_timeout = frame_timeout * 0.3333333f;
  if (age > one_third_timeout) {
    Ogre::ColourValue grey(0.7f, 0.7f, 0.7f, 1.0f);

    if (age > one_third_timeout * 2) {
      double a = std::max(0.0, (frame_timeout - age) / one_third_timeout);
      Ogre::ColourValue c = Ogre::ColourValue(grey.r, grey.g, grey.b, a);

      axes_->setXColor(c);
      axes_->setYColor(c);
      axes_->setZColor(c);
      name_text_->setColor(c);
      parent_arrow_->setColor(c.r, c.g, c.b, c.a);
    } else {
      double t = std::max(0.0, (one_third_timeout * 2 - age) / one_third_timeout);
      axes_->setXColor(lerpColor(axes_->getDefaultXColor(), grey, t));
      axes_->setYColor(lerpColor(axes_->getDefaultYColor(), grey, t));
      axes_->setZColor(lerpColor(axes_->getDefaultZColor(), grey, t));
      name_text_->setColor(lerpColor(Ogre::ColourValue::White, grey, t));
      parent_arrow_->setShaftColor(lerpColor(ARROW_SHAFT_COLOR, grey, t));
      parent_arrow_->setHeadColor(lerpColor(ARROW_HEAD_COLOR, grey, t));
    }
  } else {
    axes_->setToDefaultColors();
    name_text_->setColor(Ogre::ColourValue::White);
    parent_arrow_->setHeadColor(ARROW_HEAD_COLOR);
    parent_arrow_->setShaftColor(ARROW_SHAFT_COLOR);
  }
}

void FrameInfo::updateParentArrow(
  const Ogre::Vector3 & position,
  const Ogre::Vector3 & parent_position,
  const float scale)
{
  Ogre::Vector3 direction = parent_position - position;
  float distance = direction.length();
  direction.normalise();

  Ogre::Quaternion orient = Ogre::Vector3::NEGATIVE_UNIT_Z.getRotationTo(direction);

  if (direction.squaredLength() > 0 && !orient.isNaN()) {
    setParentArrowVisible(true);
    distance_to_parent_ = distance;
    float head_length = (distance < 0.1f * scale) ? (0.1f * scale * distance) : 0.1f * scale;
    float shaft_length = distance - head_length;
    // aleeper: This was changed from 0.02 and 0.08 to 0.01 and 0.04
    // to match proper radius handling in arrow.cpp
    parent_arrow_->set(shaft_length, 0.01f * scale, head_length, 0.04f * scale);

    parent_arrow_->setPosition(position);
    parent_arrow_->setOrientation(orient);
  } else {
    setParentArrowVisible(false);
  }
}

}  // namespace displays
}  // namespace rviz_default_plugins
