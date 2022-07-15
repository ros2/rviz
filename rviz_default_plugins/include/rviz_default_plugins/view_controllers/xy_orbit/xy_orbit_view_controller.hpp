/*
 * Copyright (c) 2009, Willow Garage, Inc.
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

#ifndef RVIZ_DEFAULT_PLUGINS__VIEW_CONTROLLERS__XY_ORBIT__XY_ORBIT_VIEW_CONTROLLER_HPP_
#define RVIZ_DEFAULT_PLUGINS__VIEW_CONTROLLERS__XY_ORBIT__XY_ORBIT_VIEW_CONTROLLER_HPP_

#include <utility>

#ifdef __clang__
# pragma clang diagnostic push
# pragma clang diagnostic ignored "-Wkeyword-macro"
#endif

#include <OgreRay.h>
#include <OgreVector.h>

#ifdef __clang__
# pragma clang diagnostic pop
#endif

#include "rviz_default_plugins/view_controllers/orbit/orbit_view_controller.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

namespace Ogre
{
class SceneNode;
}

namespace rviz_default_plugins
{
namespace view_controllers
{
/**
 * \brief Like the orbit view controller, but focal point moves only in the x-y plane.
 */
class RVIZ_DEFAULT_PLUGINS_PUBLIC XYOrbitViewController : public OrbitViewController
{
  Q_OBJECT

public:
  void onInitialize() override;

  void lookAt(const Ogre::Vector3 & point) override;

  /** @brief Configure the settings of this view controller to give,
   * as much as possible, a similar view as that given by the
   * @param source_view.
   *
   * @param source_view must return a valid Ogre::Camera* from getCamera().
   */
  void mimic(ViewController * source_view) override;

protected:
  void updateCamera() override;

  void moveFocalPoint(
    float distance, int32_t diff_x, int32_t diff_y, int32_t last_x, int32_t last_y) override;

  std::pair<bool, Ogre::Vector3> intersectGroundPlane(Ogre::Ray mouse_ray);

  void handleWheelEvent(rviz_common::ViewportMouseEvent & event, float distance) override;

  void handleRightClick(
    rviz_common::ViewportMouseEvent & event, float distance, int32_t diff_y) override;

  void setShiftOrbitStatus() override;

  void setNewFocalPointKeepingViewIfPossible(ViewController * source_view);

  void calculateNewCameraPositionAndOrientation(
    const Ogre::Camera * source_camera,
    Ogre::Ray & camera_dir_ray);
};


}  // namespace view_controllers
}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__VIEW_CONTROLLERS__XY_ORBIT__XY_ORBIT_VIEW_CONTROLLER_HPP_
