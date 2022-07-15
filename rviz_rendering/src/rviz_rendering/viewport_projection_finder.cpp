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

#include "rviz_rendering/viewport_projection_finder.hpp"

#include <utility>

#include <OgreCamera.h>
#include <OgrePlane.h>
#include <OgreRay.h>
#include <OgreVector.h>
#include <OgreViewport.h>

#include "rviz_rendering/render_window.hpp"

namespace rviz_rendering
{

std::pair<bool, Ogre::Vector3> ViewportProjectionFinder::getViewportPointProjectionOnXYPlane(
  RenderWindow * render_window, int x, int y)
{
  auto xy_plane = Ogre::Plane(Ogre::Vector3::UNIT_Z, 0.0f);
  return getViewportProjectionOnPlane(render_window, x, y, xy_plane);
}

std::pair<bool, Ogre::Vector3> ViewportProjectionFinder::getViewportProjectionOnPlane(
  RenderWindow * render_window, int x, int y, Ogre::Plane & plane)
{
  auto viewport = RenderWindowOgreAdapter::getOgreViewport(render_window);
  int width = viewport->getActualWidth();
  int height = viewport->getActualHeight();
  Ogre::Ray mouse_ray = viewport->getCamera()->getCameraToViewportRay(
    static_cast<float>(x) / static_cast<float>(width),
    static_cast<float>(y) / static_cast<float>(height));

  auto intersection = mouse_ray.intersects(plane);
  if (!intersection.first) {
    return {false, Ogre::Vector3()};
  }

  auto intersection_point = mouse_ray.getPoint(intersection.second);
  return {true, intersection_point};
}

}  // namespace rviz_rendering
