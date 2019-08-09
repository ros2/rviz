/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * Copyright (c) 2017, Open Source Robotics Foundation, Inc.
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

#include "rviz_common/frame_position_tracking_view_controller.hpp"

#include <OgreCamera.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include "rviz_common/display_context.hpp"
#include "frame_manager.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/tf_frame_property.hpp"
#include "rviz_common/view_manager.hpp"
#include "rviz_common/viewport_mouse_event.hpp"

namespace rviz_common
{

FramePositionTrackingViewController::FramePositionTrackingViewController()
: target_scene_node_(nullptr),
  reference_orientation_(Ogre::Quaternion::IDENTITY),
  reference_position_(Ogre::Vector3::ZERO),
  camera_scene_node_(nullptr)
{
  target_frame_property_ = new rviz_common::properties::TfFrameProperty(
    "Target Frame",
    rviz_common::properties::TfFrameProperty::FIXED_FRAME_STRING,
    "TF frame whose motion this view will follow.",
    this,
    nullptr,
    true);
}

void FramePositionTrackingViewController::onInitialize()
{
  target_frame_property_->setFrameManager(context_->getFrameManager());

  target_scene_node_ = context_->getSceneManager()->getRootSceneNode()->createChildSceneNode();
  camera_scene_node_ = target_scene_node_->createChildSceneNode();

  camera_->detachFromParent();
  camera_scene_node_->attachObject(camera_);
}

FramePositionTrackingViewController::~FramePositionTrackingViewController()
{
  context_->getSceneManager()->destroySceneNode(target_scene_node_);
}

void FramePositionTrackingViewController::onActivate()
{
  updateTargetSceneNode();

  // Before activation, changes to target frame property should have
  // no side-effects.  After activation, changing target frame
  // property has the side effect (typically) of changing an offset
  // property so that the view does not jump.  Therefore we make the
  // signal/slot connection from the property here in onActivate()
  // instead of in the constructor.
  connect(target_frame_property_, SIGNAL(changed()), this, SLOT(updateTargetFrame()));
}

void FramePositionTrackingViewController::update(float dt, float ros_dt)
{
  Q_UNUSED(dt);
  Q_UNUSED(ros_dt);
  updateTargetSceneNode();
}

void FramePositionTrackingViewController::updateTargetFrame()
{
  Ogre::Vector3 old_position = target_scene_node_->getPosition();
  Ogre::Quaternion old_orientation = target_scene_node_->getOrientation();

  updateTargetSceneNode();

  onTargetFrameChanged(old_position, old_orientation);
}

void FramePositionTrackingViewController::onTargetFrameChanged(
  const Ogre::Vector3 & old_reference_position,
  const Ogre::Quaternion & old_reference_orientation)
{
  Q_UNUSED(old_reference_position);
  Q_UNUSED(old_reference_orientation);
}

bool FramePositionTrackingViewController::getNewTransform()
{
  Ogre::Vector3 new_reference_position;
  Ogre::Quaternion new_reference_orientation;

  bool got_transform = context_->getFrameManager()->getTransform(
    target_frame_property_->getFrameStd(), new_reference_position, new_reference_orientation);
  if (got_transform) {
    reference_position_ = new_reference_position;
    reference_orientation_ = new_reference_orientation;
  }
  return got_transform;
}

void FramePositionTrackingViewController::updateTargetSceneNode()
{
  if (getNewTransform()) {
    target_scene_node_->setPosition(reference_position_);

    context_->queueRender();
  }
}

void FramePositionTrackingViewController::mimic(ViewController * source_view)
{
  QVariant target_frame = source_view->subProp("Target Frame")->getValue();
  if (target_frame.isValid()) {
    target_frame_property_->setValue(target_frame);
    getNewTransform();
  }
}

}  // namespace rviz_common
