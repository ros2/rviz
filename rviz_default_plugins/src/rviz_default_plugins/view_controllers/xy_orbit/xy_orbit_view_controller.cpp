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

#include "rviz_default_plugins/view_controllers/xy_orbit/xy_orbit_view_controller.hpp"

#include <cstdint>
#include <utility>

#include <OgreCamera.h>
#include <OgreSceneNode.h>
#include <OgreViewport.h>

#include "rviz_rendering/objects/shape.hpp"
#include "rviz_rendering/render_window.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/vector_property.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/viewport_mouse_event.hpp"

namespace rviz_default_plugins
{
namespace view_controllers
{

// move camera up so the focal point appears in the lower image half
static const float CAMERA_OFFSET = 0.2f;

void XYOrbitViewController::onInitialize()
{
  OrbitViewController::onInitialize();
  focal_shape_->setColor(0.0f, 1.0f, 1.0f, 0.5f);
}

void XYOrbitViewController::mimic(ViewController * source_view)
{
  FramePositionTrackingViewController::mimic(source_view);

  setNewFocalPointKeepingViewIfPossible(source_view);
}

void XYOrbitViewController::setNewFocalPointKeepingViewIfPossible(ViewController * source_view)
{
  if (source_view->getClassId() == "rviz_default_plugins/TopDownOrtho") {
    Ogre::Vector3 position = mimicTopDownViewController(source_view);
    calculatePitchYawFromPosition(position);
    return;
  }

  Ogre::Camera * source_camera = source_view->getCamera();

  Ogre::Ray camera_dir_ray(source_camera->getRealPosition(), source_camera->getRealDirection());
  Ogre::Ray camera_down_ray(source_camera->getRealPosition(), -1.0f * source_camera->getRealUp());

  auto camera_intersection = intersectGroundPlane(camera_dir_ray);
  auto camera_down_intersection = intersectGroundPlane(camera_down_ray);

  if (camera_intersection.first && camera_down_intersection.first) {
    // Set a focal point by intersecting with the ground plane from above. This will be possible
    // if some part of the ground plane is visible in the view and the camera is above the z-plane.
    float l_b = source_camera->getRealPosition().distance(camera_intersection.second);
    float l_a = source_camera->getRealPosition().distance(camera_down_intersection.second);

    distance_property_->setFloat((l_b * l_a) / (CAMERA_OFFSET * l_b + l_a));
    calculateNewCameraPositionAndOrientation(source_camera, camera_dir_ray);
  }
  // If the camera is below the plane, directly above the scene (TopDownOrtho), the z plane is not
  // visible or the camera is located on the z-plane, reset to the default view, as no sensible
  // focal point can be set that "mimics" such a camera.
}

void
XYOrbitViewController::calculateNewCameraPositionAndOrientation(
  const Ogre::Camera * source_camera,
  Ogre::Ray & camera_dir_ray)
{
  Ogre::Vector3 position_offset =
    source_camera->getRealUp() * distance_property_->getFloat() * CAMERA_OFFSET;

  camera_dir_ray.setOrigin(source_camera->getRealPosition() - position_offset);
  auto new_focal_point = intersectGroundPlane(camera_dir_ray);
  focal_point_property_->setVector(new_focal_point.second - reference_position_);

  calculatePitchYawFromPosition(
    source_camera->getParentSceneNode()->getPosition() - position_offset);
}

void XYOrbitViewController::updateCamera()
{
  OrbitViewController::updateCamera();
  camera_->getParentSceneNode()->setPosition(
    camera_->getParentSceneNode()->getPosition() +
    camera_->getParentSceneNode()->getLocalAxes() * Ogre::Vector3::UNIT_Y *
    distance_property_->getFloat() * CAMERA_OFFSET);
}

void XYOrbitViewController::lookAt(const Ogre::Vector3 & point)
{
  Ogre::Vector3 camera_position = camera_->getParentSceneNode()->getPosition();
  Ogre::Vector3 new_focal_point =
    target_scene_node_->getOrientation().Inverse() * (point - target_scene_node_->getPosition());
  new_focal_point.z = 0;
  distance_property_->setFloat(new_focal_point.distance(camera_position));
  focal_point_property_->setVector(new_focal_point);

  calculatePitchYawFromPosition(camera_position);
}

void XYOrbitViewController::setShiftOrbitStatus()
{
  setStatus("<b>Left-Click:</b> Move X/Y.  <b>Right-Click:</b> Zoom.");
}

void XYOrbitViewController::moveFocalPoint(
  float distance, int32_t diff_x, int32_t diff_y, int32_t last_x, int32_t last_y)
{
  (void) distance;

  setCursor(MoveXY);
  int width = camera_->getViewport()->getActualWidth();
  int height = camera_->getViewport()->getActualHeight();

  Ogre::Ray mouse_ray = camera_->getCameraToViewportRay(
    (last_x + diff_x) / static_cast<float>(width), (last_y + diff_y) / static_cast<float>(height));

  Ogre::Ray last_mouse_ray = camera_->getCameraToViewportRay(
    last_x / static_cast<float>(width), last_y / static_cast<float>(height));

  auto last_intersect = intersectGroundPlane(last_mouse_ray);
  auto intersect = intersectGroundPlane(mouse_ray);
  if (last_intersect.first && intersect.first) {
    Ogre::Vector3 motion = last_intersect.second - intersect.second;

    // When dragging near the horizon, the motion can get out of control.
    // This throttles it to an arbitrary limit per mouse event.
    float motion_distance_limit = 1;  /*meter*/
    if (motion.length() > motion_distance_limit) {
      motion.normalise();
      motion *= motion_distance_limit;
    }

    focal_point_property_->add(motion);
    emitConfigChanged();
  }
}

std::pair<bool, Ogre::Vector3> XYOrbitViewController::intersectGroundPlane(Ogre::Ray mouse_ray)
{
  // convert rays into reference frame
  mouse_ray.setOrigin(target_scene_node_->convertWorldToLocalPosition(mouse_ray.getOrigin()));
  mouse_ray.setDirection(
    target_scene_node_->convertWorldToLocalOrientation(Ogre::Quaternion::IDENTITY) *
    mouse_ray.getDirection());

  Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, 0);

  std::pair<bool, Ogre::Real> intersection = mouse_ray.intersects(ground_plane);
  return std::make_pair(intersection.first, mouse_ray.getPoint(intersection.second));
}

void XYOrbitViewController::handleRightClick(
  rviz_common::ViewportMouseEvent & event, float distance, int32_t diff_y)
{
  (void) event;
  setCursor(Zoom);
  zoom(-diff_y * 0.1f * (distance / 10.0f));
}

void XYOrbitViewController::handleWheelEvent(
  rviz_common::ViewportMouseEvent & event, float distance)
{
  int diff = event.wheel_delta;
  zoom(diff * 0.001f * distance);
}

}  // namespace view_controllers
}  // namespace rviz_default_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  rviz_default_plugins::view_controllers::XYOrbitViewController,
  rviz_common::ViewController)
