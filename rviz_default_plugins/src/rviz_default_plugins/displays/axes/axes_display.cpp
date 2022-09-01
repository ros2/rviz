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

#include "rviz_default_plugins/displays/axes/axes_display.hpp"

#include <memory>
#include <string>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include "rviz_rendering/objects/axes.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/frame_manager_iface.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/tf_frame_property.hpp"


namespace rviz_default_plugins
{
namespace displays
{

AxesDisplay::AxesDisplay()
: Display(), axes_(nullptr)
{
  frame_property_ = new rviz_common::properties::TfFrameProperty(
    "Reference Frame",
    rviz_common::properties::TfFrameProperty::FIXED_FRAME_STRING,
    "The TF frame these axes will use for their origin.",
    this, nullptr, true);

  length_property_ = new rviz_common::properties::FloatProperty(
    "Length", 1.0f, "Length of each axis, in meters.", this, SLOT(updateShape()));
  length_property_->setMin(0.0001f);

  radius_property_ = new rviz_common::properties::FloatProperty(
    "Radius", 0.1f, "Radius of each axis, in meters.", this, SLOT(updateShape()));
  radius_property_->setMin(0.0001f);
}

AxesDisplay::~AxesDisplay() = default;

void AxesDisplay::onInitialize()
{
  frame_property_->setFrameManager(context_->getFrameManager());

  axes_ = std::make_shared<rviz_rendering::Axes>(
    scene_manager_, scene_node_, length_property_->getFloat(), radius_property_->getFloat());
  axes_->getSceneNode()->setVisible(isEnabled());
}

void AxesDisplay::onEnable()
{
  axes_->getSceneNode()->setVisible(true);
}

void AxesDisplay::onDisable()
{
  axes_->getSceneNode()->setVisible(false);
}

void AxesDisplay::updateShape()
{
  axes_->set(length_property_->getFloat(), radius_property_->getFloat());
  context_->queueRender();
}

void AxesDisplay::update(float dt, float ros_dt)
{
  (void) dt;
  (void) ros_dt;

  std::string frame = frame_property_->getFrameStd();

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (context_->getFrameManager()->getTransform(
      frame, position, orientation))
  {
    axes_->getSceneNode()->setVisible(true);
    axes_->setPosition(position);
    axes_->setOrientation(orientation);
    setStatus(rviz_common::properties::StatusProperty::Ok, "Transform", "Transform OK");
  } else {
    std::string error;
    if (context_->getFrameManager()->transformHasProblems(
        frame, context_->getClock()->now(), error))
    {
      setStatus(
        rviz_common::properties::StatusProperty::Error, "Transform", QString::fromStdString(error));
    } else {
      setMissingTransformToFixedFrame(frame);
    }
    axes_->getSceneNode()->setVisible(false);
  }
}

}  // namespace displays
}  // namespace rviz_default_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::AxesDisplay, rviz_common::Display)
