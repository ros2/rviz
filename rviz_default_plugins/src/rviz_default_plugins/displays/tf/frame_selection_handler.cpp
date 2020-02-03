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

#include "rviz_default_plugins/displays/tf/frame_selection_handler.hpp"

#include <string>

#include "rviz_common/frame_manager_iface.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/quaternion_property.hpp"
#include "rviz_common/properties/string_property.hpp"
#include "rviz_common/properties/vector_property.hpp"

using rviz_common::interaction::SelectionHandler;
using rviz_common::properties::BoolProperty;
using rviz_common::properties::FloatProperty;
using rviz_common::properties::StatusProperty;
using rviz_common::properties::StringProperty;
using rviz_common::properties::Property;
using rviz_common::properties::QuaternionProperty;
using rviz_common::properties::VectorProperty;
using rviz_common::interaction::Picked;

namespace rviz_default_plugins
{
namespace displays
{

FrameSelectionHandler::FrameSelectionHandler(
  FrameInfo * frame,
  TFDisplay * display,
  rviz_common::DisplayContext * context)
: SelectionHandler(context),
  frame_(frame),
  display_(display),
  category_property_(nullptr),
  enabled_property_(nullptr),
  parent_property_(nullptr),
  position_property_(nullptr),
  orientation_property_(nullptr)
{
}

void FrameSelectionHandler::createProperties(const Picked & obj, Property * parent_property)
{
  (void) obj;
  (void) display_;
  category_property_ = new Property(
    "Frame " + QString::fromStdString(frame_->name_),
    QVariant(), "", parent_property);

  enabled_property_ =
    new BoolProperty(
    "Enabled", true, "", category_property_, SLOT(
      updateVisibilityFromSelection()), frame_);

  parent_property_ = new StringProperty("Parent", "", "", category_property_);
  parent_property_->setReadOnly(true);

  position_property_ = new VectorProperty("Position", Ogre::Vector3::ZERO, "", category_property_);
  position_property_->setReadOnly(true);

  orientation_property_ = new QuaternionProperty(
    "Orientation", Ogre::Quaternion::IDENTITY, "",
    category_property_);
  orientation_property_->setReadOnly(true);
}

void FrameSelectionHandler::destroyProperties(const Picked & obj, Property * parent_property)
{
  (void) obj;
  (void) parent_property;
  delete category_property_;  // This deletes its children as well.
  category_property_ = nullptr;
  enabled_property_ = nullptr;
  parent_property_ = nullptr;
  position_property_ = nullptr;
  orientation_property_ = nullptr;
}

bool FrameSelectionHandler::getEnabled()
{
  if (enabled_property_) {
    return enabled_property_->getBool();
  }
  return false;  // should never happen, but don't want to crash if it does.
}

void FrameSelectionHandler::setEnabled(bool enabled)
{
  if (enabled_property_) {
    enabled_property_->setBool(enabled);
  }
}

void FrameSelectionHandler::setParentName(std::string parent_name)
{
  if (parent_property_) {
    parent_property_->setStdString(parent_name);
  }
}

void FrameSelectionHandler::setPosition(const Ogre::Vector3 & position)
{
  if (position_property_) {
    position_property_->setVector(position);
  }
}

void FrameSelectionHandler::setOrientation(const Ogre::Quaternion & orientation)
{
  if (orientation_property_) {
    orientation_property_->setQuaternion(orientation);
  }
}

}  // namespace displays

}  // namespace rviz_default_plugins
