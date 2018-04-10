/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * Copyright (c) 2017, Open Source Robotics Foundation, Inc.
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

#ifndef RVIZ_DEFAULT_PLUGINS__VIEW_CONTROLLERS__ORBIT__ORBIT_VIEW_CONTROLLER_HPP_
#define RVIZ_DEFAULT_PLUGINS__VIEW_CONTROLLERS__ORBIT__ORBIT_VIEW_CONTROLLER_HPP_

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# pragma GCC diagnostic ignored "-Wpedantic"
#endif

#include <OgreVector3.h>

#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include <QCursor>

#include "rviz_common/frame_position_tracking_view_controller.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/vector_property.hpp"
#include "rviz_rendering/objects/shape.hpp"

namespace rviz_default_plugins
{
namespace view_controllers
{

/// An orbital camera, controlled by yaw, pitch, distance, and focal point
/**
 * This camera is based on the equation of a sphere in spherical coordinates:
 *
 *   x = d * cos(theta) * sin(phi)
 *   y = d * cos(phi)
 *   z = d * sin(theta) * sin(phi)
 *
 * Where:
 *
 *   d = distance
 *   theta = yaw
 *   phi = pitch
 */
class OrbitViewController : public rviz_common::FramePositionTrackingViewController
{
  Q_OBJECT

public:
  OrbitViewController();
  virtual ~OrbitViewController();

  /// Do subclass-specific initialization.
  /**
   * Called by ViewController::initialize after context_, target_scene_node_,
   * and camera_ are set.
   */
  void onInitialize() override;

  /// Move in/out from the focal point, i.e. adjust distance by amount.
  /**
   * Positive amount moves towards the focal point, negative moves away.
   *
   * \param amount The distance to move.
   */
  void zoom(float amount);

  /// Set the yaw angle.
  void yaw(float angle);

  /// Set the pitch angle.
  void pitch(float angle);

  /// Move the focal point.
  void move(float x, float y, float z);  // NOLINT(build/include_what_you_use): not std::move

  /// Handle incoming mouse events.
  void handleMouseEvent(rviz_common::ViewportMouseEvent & evt) override;

  /// Look at a given location by changing the distance and angles.
  void lookAt(const Ogre::Vector3 & point) override;

  /// Reset the distance and angles to their default values.
  void reset() override;

  /// Configure this view controller to give a similar view to the given source_view.
  /**
   * \param source_view must return a valid `Ogre::Camera *` from `getCamera()`.
   */
  void mimic(ViewController * source_view) override;

protected:
  void update(float dt, float ros_dt) override;

  void onTargetFrameChanged(
    const Ogre::Vector3 & old_reference_position,
    const Ogre::Quaternion & old_reference_orientation) override;

  /// Calculate the pitch and yaw values given a new position and the current focal point.
  /**
   * \param position the position from which to calculate the pitch/yaw.
   */
  void calculatePitchYawFromPosition(const Ogre::Vector3 & position);

  /// Calculate the focal shape size and update it's geometry.
  void updateFocalShapeSize();

  virtual void updateCamera();

  /// The camera's yaw (rotation around the y-axis), in radians.
  rviz_common::properties::FloatProperty * yaw_property_;
  /// The camera's pitch (rotation around the x-axis), in radians.
  rviz_common::properties::FloatProperty * pitch_property_;
  /// The camera's distance from the focal point.
  rviz_common::properties::FloatProperty * distance_property_;
  /// The point around which the camera "orbits".
  rviz_common::properties::VectorProperty * focal_point_property_;
  /// Whether the focal shape size is fixed or not.
  rviz_common::properties::BoolProperty * focal_shape_fixed_size_property_;
  /// The focal shape size.
  rviz_common::properties::FloatProperty * focal_shape_size_property_;

  rviz_rendering::Shape * focal_shape_;

  bool dragging_;
};

}  // namespace view_controllers
}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__VIEW_CONTROLLERS__ORBIT__ORBIT_VIEW_CONTROLLER_HPP_
