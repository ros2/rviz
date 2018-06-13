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

#include "fps_view_controller.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# pragma GCC diagnostic ignored "-Wpedantic"
#endif
#include <OgreCamera.h>
#include <OgreQuaternion.h>
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreVector3.h>
#include <OgreViewport.h>
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "rviz_rendering/geometry.hpp"
#include "rviz_rendering/objects/shape.hpp"

#include "rviz_common/uniform_string_stream.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/viewport_mouse_event.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/vector_property.hpp"

namespace rviz_default_plugins
{
namespace view_controllers
{

static const Ogre::Quaternion ROBOT_TO_CAMERA_ROTATION =
  Ogre::Quaternion(Ogre::Radian(-Ogre::Math::HALF_PI), Ogre::Vector3::UNIT_Y) *
  Ogre::Quaternion(Ogre::Radian(-Ogre::Math::HALF_PI), Ogre::Vector3::UNIT_Z);

static const float PITCH_LIMIT_LOW = -Ogre::Math::HALF_PI + 0.001f;
static const float PITCH_LIMIT_HIGH = Ogre::Math::HALF_PI - 0.001f;

FPSViewController::FPSViewController()
{
  yaw_property_ = new rviz_common::properties::FloatProperty(
    "Yaw", 0, "Rotation of the camera around the Z (up) axis.", this);

  pitch_property_ = new rviz_common::properties::FloatProperty(
    "Pitch", 0, "How much the camera is tipped downward.", this);
  pitch_property_->setMax(Ogre::Math::HALF_PI - 0.001f);
  pitch_property_->setMin(-pitch_property_->getMax());

  position_property_ = new rviz_common::properties::VectorProperty(
    "Position", Ogre::Vector3(5, 5, 10), "Position of the camera.", this);
}

FPSViewController::~FPSViewController() = default;

void FPSViewController::onInitialize()
{
  FramePositionTrackingViewController::onInitialize();
  camera_->setProjectionType(Ogre::PT_PERSPECTIVE);
  invert_z_->hide();
}

void FPSViewController::reset()
{
  // TODO(Martin-Idel-SI): Redo...
  camera_scene_node_->setPosition(Ogre::Vector3(5, 5, 10));
  // camera_scene_node_->lookAt(0, 0, 0);
  setPropertiesFromCamera(camera_);

  // Hersh says: why is the following junk necessary?  I don't know.
  // However, without this you need to call reset() twice after
  // switching from TopDownOrtho to FPS.  After the first call the
  // camera is in the right position but pointing the wrong way.
  updateCamera();
  // camera_scene_node_->lookAt(Ogre::Vector3::ZERO, Ogre::TransformSpace::Local);
  setPropertiesFromCamera(camera_);
}

void FPSViewController::handleMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  if (event.shift()) {
    setStatus("<b>Left-Click:</b> Move X/Y.  <b>Right-Click:</b>: Move Z.");
  } else {
    setStatus(
      "<b>Left-Click:</b> Rotate.  "
      "<b>Middle-Click:</b> Move X/Y.  "
      "<b>Right-Click:</b>: Zoom.  "
      "<b>Shift</b>: More options.");
  }

  bool moved = false;

  int32_t diff_x = 0;
  int32_t diff_y = 0;

  if (event.type == QEvent::MouseMove) {
    diff_x = event.x - event.last_x;
    diff_y = event.y - event.last_y;
    moved = true;
  }

  if (event.left() && !event.shift()) {
    setCursor(Rotate3D);
    yaw(-diff_x * 0.005f);
    pitch(diff_y * 0.005f);
  } else if (event.middle() || (event.shift() && event.left())) {
    setCursor(MoveXY);
    move(diff_x * 0.01f, -diff_y * 0.01f, 0.0f);
  } else if (event.right()) {
    setCursor(MoveZ);
    move(0.0f, 0.0f, diff_y * 0.1f);
  } else {
    setCursor(event.shift() ? MoveXY : Rotate3D);
  }

  if (event.wheel_delta != 0) {
    int diff = event.wheel_delta;
    move(0.0f, 0.0f, -diff * 0.01f);

    moved = true;
  }

  if (moved) {
    context_->queueRender();
  }
}

void FPSViewController::setPropertiesFromCamera(Ogre::Camera * source_camera)
{
  auto source_parent = source_camera->getParentSceneNode();
  Ogre::Quaternion quat = source_parent->getOrientation() * ROBOT_TO_CAMERA_ROTATION.Inverse();
  float yaw = quat.getRoll(false).valueRadians();  // OGRE camera frame looks along -Z, so they
  // call rotation around Z "roll".
  float pitch = quat.getYaw(false).valueRadians();  // OGRE camera frame has +Y as "up", so they
  // call rotation around Y "yaw".

  Ogre::Vector3 direction = quat * Ogre::Vector3::NEGATIVE_UNIT_Z;

  if (direction.dotProduct(Ogre::Vector3::NEGATIVE_UNIT_Z) < 0) {
    if (pitch > Ogre::Math::HALF_PI) {
      pitch -= Ogre::Math::PI;
    } else if (pitch < -Ogre::Math::HALF_PI) {
      pitch += Ogre::Math::PI;
    }

    yaw = -yaw;

    if (direction.dotProduct(Ogre::Vector3::UNIT_X) < 0) {
      yaw -= Ogre::Math::PI;
    } else {
      yaw += Ogre::Math::PI;
    }
  }

  pitch_property_->setFloat(pitch);
  yaw_property_->setFloat(rviz_rendering::mapAngleTo0_2Pi(yaw));
  position_property_->setVector(source_parent->getPosition());
}

void FPSViewController::mimic(rviz_common::ViewController * source_view)
{
  FramePositionTrackingViewController::mimic(source_view);
  setPropertiesFromCamera(source_view->getCamera());
}

void FPSViewController::update(float dt, float ros_dt)
{
  FramePositionTrackingViewController::update(dt, ros_dt);
  updateCamera();
}

void FPSViewController::lookAt(const Ogre::Vector3 & point)
{
  camera_->lookAt(point);
  setPropertiesFromCamera(camera_);
}

void FPSViewController::onTargetFrameChanged(
  const Ogre::Vector3 & old_reference_position, const Ogre::Quaternion & old_reference_orientation)
{
  (void) old_reference_orientation;
  position_property_->add(old_reference_position - reference_position_);
}

void FPSViewController::updateCamera()
{
  camera_scene_node_->setOrientation(getOrientation());
  camera_scene_node_->setPosition(position_property_->getVector());
}

void FPSViewController::yaw(float angle)
{
  yaw_property_->setFloat(rviz_rendering::mapAngleTo0_2Pi(yaw_property_->getFloat() + angle));
}

void FPSViewController::pitch(float angle)
{
  pitch_property_->add(angle);
}

Ogre::Quaternion FPSViewController::getOrientation()
{
  Ogre::Quaternion pitch, yaw;

  yaw.FromAngleAxis(Ogre::Radian(yaw_property_->getFloat()), Ogre::Vector3::UNIT_Z);
  pitch.FromAngleAxis(Ogre::Radian(pitch_property_->getFloat()), Ogre::Vector3::UNIT_Y);

  return yaw * pitch * ROBOT_TO_CAMERA_ROTATION;
}

void FPSViewController::move(float x, float y, float z)  // NOLINT (this is not std::move)
{
  Ogre::Vector3 translate(x, y, z);
  position_property_->add(getOrientation() * translate);
}
}  // namespace view_controllers
}  // namespace rviz_default_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT(build/include_order)
PLUGINLIB_EXPORT_CLASS(
  rviz_default_plugins::view_controllers::FPSViewController,
  rviz_common::ViewController)
