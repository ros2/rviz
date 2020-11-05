/*
 * Copyright (c) 2009, Willow Garage, Inc.
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

#include "rviz_default_plugins/view_controllers/ortho/fixed_orientation_ortho_view_controller.hpp"

#include <utility>

#include <OgreCamera.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreViewport.h>

#include "rviz_common/display_context.hpp"
#include "rviz_rendering/orthographic.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/viewport_mouse_event.hpp"
#include "rviz_rendering/objects/shape.hpp"

namespace rviz_default_plugins
{
namespace view_controllers
{

static const float ORTHO_VIEW_CONTROLLER_CAMERA_Z = 500;

FixedOrientationOrthoViewController::FixedOrientationOrthoViewController()
: dragging_(false)
{
  scale_property_ = new rviz_common::properties::FloatProperty(
    "Scale", 10, "How much to scale up the size of things in the scene.", this);
  angle_property_ = new rviz_common::properties::FloatProperty(
    "Angle", 0, "Angle around the Z axis to rotate.", this);
  x_property_ = new rviz_common::properties::FloatProperty(
    "X", 0, "X component of camera position.", this);
  y_property_ = new rviz_common::properties::FloatProperty(
    "Y", 0, "Y component of camera position.", this);
}

void FixedOrientationOrthoViewController::onInitialize()
{
  FramePositionTrackingViewController::onInitialize();

  camera_->setProjectionType(Ogre::PT_ORTHOGRAPHIC);
  auto camera_parent = getCameraParent(camera_);
  camera_parent->setFixedYawAxis(false);
  invert_z_->hide();
}

void FixedOrientationOrthoViewController::reset()
{
  scale_property_->setFloat(10);
  angle_property_->setFloat(0);
  x_property_->setFloat(0);
  y_property_->setFloat(0);
}

void FixedOrientationOrthoViewController::handleMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  if (event.shift()) {
    setStatus("<b>Left-Click:</b> Move X/Y.");
  } else {
    setStatus(
      "<b>Left-Click:</b> Rotate.  <b>Middle-Click:</b> Move X/Y. "
      " <b>Right-Click:</b> Zoom.  <b>Shift:</b> More options.");
  }

  int32_t diff_x = 0;
  int32_t diff_y = 0;

  if (event.type == QEvent::MouseButtonPress) {
    dragging_ = true;
  } else if (event.type == QEvent::MouseButtonRelease) {
    dragging_ = false;
  } else if (dragging_ && event.type == QEvent::MouseMove) {
    diff_x = event.x - event.last_x;
    diff_y = event.y - event.last_y;
    renderOnMove();
  }

  if (event.left() && !event.shift()) {
    setCursor(Rotate2D);
    angle_property_->add(diff_x * 0.005f);
    orientCamera();
  } else if (event.middle() || (event.shift() && event.left())) {
    setCursor(MoveXY);
    float scale = scale_property_->getFloat();
    move(-diff_x / scale, diff_y / scale);
  } else if (event.right()) {
    setCursor(Zoom);
    scale_property_->multiply(1.0f - diff_y * 0.01f);
  } else {
    setCursor(event.shift() ? MoveXY : Rotate2D);
  }

  if (event.wheel_delta != 0) {
    int diff = event.wheel_delta;
    scale_property_->multiply(1.0f - (-diff) * 0.001f);
    renderOnMove();
  }
}

void FixedOrientationOrthoViewController::renderOnMove()
{
  context_->queueRender();
  emitConfigChanged();
}

void FixedOrientationOrthoViewController::orientCamera()
{
  auto camera_parent = getCameraParent(camera_);
  camera_parent->setOrientation(
    Ogre::Quaternion(Ogre::Radian(angle_property_->getFloat()), Ogre::Vector3::UNIT_Z));
}

void FixedOrientationOrthoViewController::mimic(ViewController * source_view)
{
  FramePositionTrackingViewController::mimic(source_view);

  if (source_view->getClassId() == "rviz_default_plugins/TopDownOrtho") {
    auto source_ortho = qobject_cast<FixedOrientationOrthoViewController *>(source_view);
    scale_property_->setFloat(source_ortho->scale_property_->getFloat());
    angle_property_->setFloat(source_ortho->angle_property_->getFloat());
    x_property_->setFloat(source_ortho->x_property_->getFloat());
    y_property_->setFloat(source_ortho->y_property_->getFloat());
  } else if (source_view->getFocalPointStatus().exists_) {
    setPosition(source_view->getFocalPointStatus().value_);
  } else {
    // if the previous view does not have a focal point and is not the same as this, the camera is
    // placed at (x, y, ORTHO_VIEW_CONTROLLER_CAMERA_Z), where x and y are first two coordinates of
    // the old camera position.
    auto source_camera_parent = getCameraParent(source_view->getCamera());
    setPosition(source_camera_parent->getPosition());
  }
}

void FixedOrientationOrthoViewController::update(float dt, float ros_dt)
{
  FramePositionTrackingViewController::update(dt, ros_dt);
  updateCamera();
}

void FixedOrientationOrthoViewController::lookAt(const Ogre::Vector3 & point)
{
  setPosition(point - target_scene_node_->getPosition());
}

void FixedOrientationOrthoViewController::onTargetFrameChanged(
  const Ogre::Vector3 & old_reference_position, const Ogre::Quaternion & old_reference_orientation)
{
  (void) old_reference_orientation;

  move(
    old_reference_position.x - reference_position_.x,
    old_reference_position.y - reference_position_.y);
}

void FixedOrientationOrthoViewController::updateCamera()
{
  orientCamera();

  float width = camera_->getViewport()->getActualWidth();
  float height = camera_->getViewport()->getActualHeight();

  float scale = scale_property_->getFloat();
  float ortho_width = width / scale / 2;
  float ortho_height = height / scale / 2;
  Ogre::Matrix4 projection = rviz_rendering::buildScaledOrthoMatrix(
    -ortho_width, ortho_width, -ortho_height, ortho_height,
    camera_->getNearClipDistance(), camera_->getFarClipDistance());
  camera_->setCustomProjectionMatrix(true, projection);

  // For Z, we use a value that seems to work very well in the past. It once was connected to
  // half the far_clip_distance.
  auto camera_parent = getCameraParent(camera_);
  camera_parent->setPosition(
    Ogre::Vector3(
      x_property_->getFloat(), y_property_->getFloat(), ORTHO_VIEW_CONTROLLER_CAMERA_Z));
}

Ogre::SceneNode * FixedOrientationOrthoViewController::getCameraParent(Ogre::Camera * camera)
{
  auto camera_parent = camera->getParentSceneNode();

  if (!camera_parent) {
    throw std::runtime_error("camera's parent scene node pointer unexpectedly nullptr");
  }
  return camera_parent;
}

void FixedOrientationOrthoViewController::setPosition(const Ogre::Vector3 & pos_rel_target)
{
  x_property_->setFloat(pos_rel_target.x);
  y_property_->setFloat(pos_rel_target.y);
}

void FixedOrientationOrthoViewController::move(float dx, float dy)
{
  float angle = angle_property_->getFloat();
  x_property_->add(dx * cos(angle) - dy * sin(angle));
  y_property_->add(dx * sin(angle) + dy * cos(angle));
}

}  // namespace view_controllers
}  // namespace rviz_default_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT(build/include_order)
PLUGINLIB_EXPORT_CLASS(
  rviz_default_plugins::view_controllers::FixedOrientationOrthoViewController,
  rviz_common::ViewController)
