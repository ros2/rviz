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

#ifndef RVIZ_DEFAULT_PLUGINS__VIEW_CONTROLLERS__FPS__FPS_VIEW_CONTROLLER_HPP_
#define RVIZ_DEFAULT_PLUGINS__VIEW_CONTROLLERS__FPS__FPS_VIEW_CONTROLLER_HPP_

#ifdef __clang__
# pragma clang diagnostic push
# pragma clang diagnostic ignored "-Wkeyword-macro"
#endif

#include <OgreVector.h>
#include <OgreQuaternion.h>

#ifdef __clang__
# pragma clang diagnostic pop
#endif

#include "rviz_common/frame_position_tracking_view_controller.hpp"

#include "rviz_default_plugins/visibility_control.hpp"

namespace rviz_common
{
namespace properties
{
class FloatProperty;
class Shape;
class VectorProperty;
}  // namespace properties
}  // namespace rviz_common

namespace rviz_default_plugins
{
namespace view_controllers
{
/** @brief A first-person camera, controlled by yaw, pitch, and position. */
class RVIZ_DEFAULT_PLUGINS_PUBLIC FPSViewController : public
  rviz_common::FramePositionTrackingViewController
{
public:
  FPSViewController();

  ~FPSViewController() override;

  void onInitialize() override;

  void yaw(float angle);

  void pitch(float angle);

  void move(float x, float y, float z);  // NOLINT (this is not std::move)

  void handleMouseEvent(rviz_common::ViewportMouseEvent & evt) override;

  void lookAt(const Ogre::Vector3 & point) override;

  void reset() override;

  /** @brief Configure the settings of this view controller to give,
   * as much as possible, a similar view as that given by the
   * @param source_view.
   *
   * @param source_view must return a valid @c Ogre::Camera* from getCamera(). */
  void mimic(rviz_common::ViewController * source_view) override;

  void update(float dt, float ros_dt) override;

protected:
  void onTargetFrameChanged(
    const Ogre::Vector3 & old_reference_position,
    const Ogre::Quaternion & old_reference_orientation) override;

  void setPropertiesFromCamera(Ogre::Camera * source_camera);
  void updateCamera();

  void setCursorStatus(rviz_common::ViewportMouseEvent & event);
  bool extractMouseMoveDifference(
    const rviz_common::ViewportMouseEvent & event, int32_t & diff_x, int32_t & diff_y) const;
  void moveCamera(rviz_common::ViewportMouseEvent & event, int32_t diff_x, int32_t diff_y);
  bool handleMouseWheelMovement(const rviz_common::ViewportMouseEvent & event);
  void handleQuaternionOrientationAmbiguity(
    const Ogre::Quaternion & quaternion, float & yaw, float & pitch) const;

  Ogre::Quaternion getOrientation();

  rviz_common::properties::FloatProperty * yaw_property_;
  rviz_common::properties::FloatProperty * pitch_property_;
  rviz_common::properties::VectorProperty * position_property_;
};

}  // namespace view_controllers
}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__VIEW_CONTROLLERS__FPS__FPS_VIEW_CONTROLLER_HPP_
