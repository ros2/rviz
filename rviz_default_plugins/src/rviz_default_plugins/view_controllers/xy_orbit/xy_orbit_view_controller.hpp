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

#ifndef RVIZ_DEFAULT_PLUGINS__VIEW_CONTROLLERS__XY_ORBIT__XY_ORBIT_VIEW_CONTROLLER_HPP_
#define RVIZ_DEFAULT_PLUGINS__VIEW_CONTROLLERS__XY_ORBIT__XY_ORBIT_VIEW_CONTROLLER_HPP_

#include "../orbit/orbit_view_controller.hpp"

#include <OgreRay.h>
#include <OgreVector3.h>

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
class XYOrbitViewController : public OrbitViewController
{
  Q_OBJECT

public:
  virtual void onInitialize();

  virtual void handleMouseEvent(rviz_common::ViewportMouseEvent & evt);

  virtual void lookAt(const Ogre::Vector3 & point);

  /** @brief Configure the settings of this view controller to give,
   * as much as possible, a similar view as that given by the
   * @a source_view.
   *
   * @a source_view must return a valid @c Ogre::Camera* from getCamera(). */
  virtual void mimic(ViewController * source_view);

protected:
  virtual void updateCamera();

  bool intersectGroundPlane(Ogre::Ray mouse_ray, Ogre::Vector3 & intersection_3d);
};


}  // namespace view_controllers
}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__VIEW_CONTROLLERS__XY_ORBIT__XY_ORBIT_VIEW_CONTROLLER_HPP_
