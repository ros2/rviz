/*
 * Copyright (c) 2009, Willow Garage, Inc.
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

#include "rviz_default_plugins/view_controllers/orbit/orbit_view_controller.hpp"

#include <cmath>
#include <complex>
#include <cstdint>
#include <ctgmath>
#include <memory>
#include <valarray>

#include <OgreCamera.h>
#include <OgreQuaternion.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreVector.h>
#include <OgreViewport.h>

#include "rviz_common/display_context.hpp"
#include "rviz_common/viewport_mouse_event.hpp"
#include "rviz_rendering/geometry.hpp"

static const float PITCH_START = Ogre::Math::HALF_PI * 0.5f;
static const float YAW_START = Ogre::Math::HALF_PI * 0.5f;
static const float DISTANCE_START = 10;
static const float FOCAL_SHAPE_SIZE_START = 0.05f;
static const bool FOCAL_SHAPE_FIXED_SIZE = true;
static const float ROTATION_SPEED = 0.005f;

namespace rviz_default_plugins
{
namespace view_controllers
{

using rviz_common::properties::BoolProperty;
using rviz_common::properties::FloatProperty;
using rviz_common::properties::VectorProperty;
using rviz_rendering::Shape;

OrbitViewController::OrbitViewController()
: focal_shape_(nullptr), dragging_(false)
{
  distance_property_ = new FloatProperty(
    "Distance", DISTANCE_START,
    "Distance from the focal point.", this);
  distance_property_->setMin(0.001f);

  focal_shape_size_property_ = new FloatProperty(
    "Focal Shape Size", FOCAL_SHAPE_SIZE_START,
    "Focal shape size.", this);
  focal_shape_size_property_->setMin(0.001f);

  focal_shape_fixed_size_property_ = new BoolProperty(
    "Focal Shape Fixed Size",
    FOCAL_SHAPE_FIXED_SIZE, "Focal shape size.", this);

  yaw_property_ = new FloatProperty(
    "Yaw", YAW_START,
    "Rotation of the camera around the Z (up) axis.", this);

  pitch_property_ = new FloatProperty(
    "Pitch", PITCH_START,
    "How much the camera is tipped downward.", this);
  pitch_property_->setMax(Ogre::Math::HALF_PI - 0.001f);
  pitch_property_->setMin(-pitch_property_->getMax());

  focal_point_property_ = new VectorProperty(
    "Focal Point", Ogre::Vector3::ZERO,
    "The center point which the camera orbits.", this);
}

void OrbitViewController::onInitialize()
{
  rviz_common::FramePositionTrackingViewController::onInitialize();

  camera_->setProjectionType(Ogre::PT_PERSPECTIVE);

  focal_shape_ = std::make_unique<rviz_rendering::Shape>(
    Shape::Sphere, context_->getSceneManager(), target_scene_node_);
  updateFocalShapeSize();
  focal_shape_->setColor(1.0f, 1.0f, 0.0f, 0.5f);
  focal_shape_->getRootNode()->setVisible(false);
}

OrbitViewController::~OrbitViewController() = default;

void OrbitViewController::reset()
{
  dragging_ = false;
  yaw_property_->setFloat(YAW_START);
  pitch_property_->setFloat(PITCH_START);
  distance_property_->setFloat(DISTANCE_START);
  focal_shape_size_property_->setFloat(FOCAL_SHAPE_SIZE_START);
  focal_shape_fixed_size_property_->setBool(false);
  updateFocalShapeSize();
  focal_point_property_->setVector(Ogre::Vector3::ZERO);
}

void OrbitViewController::handleMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  if (event.shift()) {
    setShiftOrbitStatus();
  } else {
    setDefaultOrbitStatus();
  }

  updateFocalShapeSize();

  int32_t diff_x = 0;
  int32_t diff_y = 0;

  bool moved = setMouseMovementFromEvent(event, diff_x, diff_y);

  float distance = distance_property_->getFloat();
  if (event.left() && !event.shift()) {
    rotateCamera(diff_x, diff_y);
  } else if (event.middle() || (event.shift() && event.left())) {
    moveFocalPoint(distance, diff_x, diff_y, 0, 0);
  } else if (event.right()) {
    handleRightClick(event, distance, diff_y);
  } else {
    setCursor(event.shift() ? MoveXY : Rotate3D);
  }

  if (event.wheel_delta != 0) {
    handleWheelEvent(event, distance);
    moved = true;
  }

  if (moved) {
    context_->queueRender();
  }
}

void OrbitViewController::setShiftOrbitStatus()
{
  setStatus(
    "<b>Left-Click:</b> Move X/Y.  "
    "<b>Right-Click:</b> Move Z.  "
    "<b>Mouse Wheel:</b> Zoom.");
}

void OrbitViewController::setDefaultOrbitStatus()
{
  setStatus(
    "<b>Left-Click:</b> Rotate.  "
    "<b>Middle-Click:</b> Move X/Y.  "
    "<b>Right-Click/Mouse Wheel:</b> Zoom.  "
    "<b>Shift:</b> More options.");
}

bool OrbitViewController::setMouseMovementFromEvent(
  const rviz_common::ViewportMouseEvent & event, int32_t & diff_x, int32_t & diff_y)
{
  if (event.type == QEvent::MouseButtonPress) {
    focal_shape_->getRootNode()->setVisible(true);
    dragging_ = true;
    return true;
  } else if (event.type == QEvent::MouseButtonRelease) {
    focal_shape_->getRootNode()->setVisible(false);
    dragging_ = false;
    return true;
  } else if (dragging_ && event.type == QEvent::MouseMove) {
    diff_x = event.x - event.last_x;
    diff_y = event.y - event.last_y;
    return true;
  }
  return false;
}

void OrbitViewController::rotateCamera(int32_t diff_x, int32_t diff_y)
{
  setCursor(Rotate3D);
  yaw(diff_x * ROTATION_SPEED);
  pitch(-diff_y * ROTATION_SPEED);
}

void OrbitViewController::moveFocalPoint(
  float distance, int32_t diff_x, int32_t diff_y, int32_t last_x, int32_t last_y)
{
  (void) last_x;
  (void) last_y;
  setCursor(MoveXY);
  float fovY = camera_->getFOVy().valueRadians();
  float fovX = 2.0f * atan(tan(fovY / 2.0f) * camera_->getAspectRatio());

  int width = camera_->getViewport()->getActualWidth();
  int height = camera_->getViewport()->getActualHeight();

  move(
    -(static_cast<float>(diff_x) / static_cast<float>(width)) * distance * tan(fovX / 2.0f) * 2.0f,
    (static_cast<float>(diff_y) / static_cast<float>(height)) * distance * tan(fovY / 2.0f) * 2.0f,
    0.0f
  );
}

void OrbitViewController::handleRightClick(
  rviz_common::ViewportMouseEvent & event, float distance, int32_t diff_y)
{
  if (event.shift()) {
    setCursor(MoveZ);
    move(0.0f, 0.0f, diff_y * 0.1f * (distance / 10.0f));
  } else {
    setCursor(Zoom);
    zoom(-diff_y * 0.1f * (distance / 10.0f));
  }
}

void OrbitViewController::handleWheelEvent(rviz_common::ViewportMouseEvent & event, float distance)
{
  int diff = event.wheel_delta;
  if (event.shift()) {
    move(0, 0, -diff * 0.001f * distance);
  } else {
    zoom(diff * 0.001f * distance);
  }
}

void OrbitViewController::mimic(rviz_common::ViewController * source_view)
{
  rviz_common::FramePositionTrackingViewController::mimic(source_view);

  Ogre::Camera * source_camera = source_view->getCamera();
  Ogre::SceneNode * camera_parent = source_camera->getParentSceneNode();

  Ogre::Vector3 position = camera_parent->getPosition();
  Ogre::Quaternion orientation = camera_parent->getOrientation();

  if (source_view->getClassId() == "rviz_default_plugins/Orbit") {
    const auto source_orbit_view = dynamic_cast<OrbitViewController *>(source_view);
    distance_property_->setFloat(source_orbit_view->distance_property_->getFloat());
    focal_point_property_->setVector(source_orbit_view->focal_point_property_->getVector());
    updateFocalShapeSize();
  } else if (source_view->getClassId() == "rviz_default_plugins/TopDownOrtho") {
    position = mimicTopDownViewController(source_view);
  } else {
    // Determine the distance from here to the reference frame, and use
    // that as the distance our focal point should be at.
    distance_property_->setFloat(position.length());
    updateFocalShapeSize();
    auto direction = orientation * (Ogre::Vector3::NEGATIVE_UNIT_Z * position.length());
    focal_point_property_->setVector(position + direction);
  }

  calculatePitchYawFromPosition(position);
}

Ogre::Vector3 OrbitViewController::mimicTopDownViewController(
  rviz_common::ViewController * view_controller)
{
  float focal_point_x = view_controller->subProp("X")->getValue().toFloat();
  float focal_point_y = view_controller->subProp("Y")->getValue().toFloat();
  distance_property_->setFloat(100);
  focal_point_property_->setVector(Ogre::Vector3(focal_point_x, focal_point_y, 0));
  updateFocalShapeSize();

  // Substract a tiny bit in the y-direction to resolve pitch/yaw ambiguity.
  return Ogre::Vector3(focal_point_x, focal_point_y - 0.0001f, 100);
}

rviz_common::FocalPointStatus OrbitViewController::getFocalPointStatus()
{
  return {true, focal_point_property_->getVector()};
}

void OrbitViewController::update(float dt, float ros_dt)
{
  rviz_common::FramePositionTrackingViewController::update(dt, ros_dt);
  updateCamera();
}

void OrbitViewController::lookAt(const Ogre::Vector3 & point)
{
  Ogre::SceneNode * camera_parent = camera_->getParentSceneNode();
  Ogre::Vector3 camera_position = camera_parent->getPosition();
  focal_point_property_->setVector(
    target_scene_node_->getOrientation().Inverse() *
    (point - target_scene_node_->getPosition()));
  distance_property_->setFloat(focal_point_property_->getVector().distance(camera_position));
  updateFocalShapeSize();
  calculatePitchYawFromPosition(camera_position);
}

void OrbitViewController::onTargetFrameChanged(
  const Ogre::Vector3 & old_reference_position, const Ogre::Quaternion & old_reference_orientation)
{
  Q_UNUSED(old_reference_orientation);
  focal_point_property_->add(old_reference_position - reference_position_);
}

void OrbitViewController::updateCamera()
{
  float distance = distance_property_->getFloat();
  float yaw = yaw_property_->getFloat();
  float pitch = pitch_property_->getFloat();
  Ogre::Vector3 camera_z = Ogre::Vector3::UNIT_Z;

  // If requested, turn the world upside down.
  if (this->invert_z_->getBool()) {
    yaw = -yaw;
    pitch = -pitch;
    camera_z = -camera_z;
  }

  Ogre::Vector3 focal_point = focal_point_property_->getVector();

  float x = distance * cos(yaw) * cos(pitch) + focal_point.x;
  float y = distance * sin(yaw) * cos(pitch) + focal_point.y;
  float z = distance * sin(pitch) + focal_point.z;

  Ogre::Vector3 pos(x, y, z);

  Ogre::SceneNode * camera_parent = camera_->getParentSceneNode();
  if (!camera_parent) {
    throw std::runtime_error("camera's parent scene node pointer unexpectedly nullptr");
  }
  camera_parent->setPosition(pos);
  camera_parent->setFixedYawAxis(true, target_scene_node_->getOrientation() * camera_z);
  camera_parent->setDirection(focal_point - pos, Ogre::SceneNode::TS_PARENT);

  focal_shape_->setPosition(focal_point);
}

void OrbitViewController::yaw(float angle)
{
  yaw_property_->setFloat(rviz_rendering::mapAngleTo0_2Pi(yaw_property_->getFloat() - angle));
}

void OrbitViewController::pitch(float angle)
{
  pitch_property_->add(-angle);
}

void OrbitViewController::calculatePitchYawFromPosition(const Ogre::Vector3 & position)
{
  Ogre::Vector3 diff = position - focal_point_property_->getVector();
  pitch_property_->setFloat(asin(diff.z / distance_property_->getFloat()));
  yaw_property_->setFloat(atan2(diff.y, diff.x));
}

void OrbitViewController::updateFocalShapeSize()
{
  const float fshape_size(focal_shape_size_property_->getFloat());
  float distance_property(distance_property_->getFloat());
  if (focal_shape_fixed_size_property_->getBool()) {
    distance_property = 1;
  }

  focal_shape_->setScale(
    Ogre::Vector3(
      fshape_size * distance_property,
      fshape_size * distance_property,
      fshape_size * distance_property / 5.0f));
}

void OrbitViewController::zoom(float amount)
{
  distance_property_->add(-amount);
  updateFocalShapeSize();
}

// no lint because this is not std::move and cpplint doesn't know...
void OrbitViewController::move(float x, float y, float z)  // NOLINT(build/include_what_you_use)
{
  Ogre::SceneNode * camera_parent = camera_->getParentSceneNode();
  focal_point_property_->add(camera_parent->getOrientation() * Ogre::Vector3(x, y, z));
}

}  // namespace view_controllers
}  // namespace rviz_default_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT(build/include_order)
PLUGINLIB_EXPORT_CLASS(
  rviz_default_plugins::view_controllers::OrbitViewController, rviz_common::ViewController)
