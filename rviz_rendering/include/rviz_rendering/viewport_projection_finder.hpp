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

#ifndef RVIZ_RENDERING__VIEWPORT_PROJECTION_FINDER_HPP_
#define RVIZ_RENDERING__VIEWPORT_PROJECTION_FINDER_HPP_

#include <utility>

#include <OgreVector.h>

#include "rviz_rendering/visibility_control.hpp"

namespace Ogre
{
class Plane;
class Viewport;
}

namespace rviz_rendering
{

class RenderWindow;

class RVIZ_RENDERING_PUBLIC ViewportProjectionFinder
{
public:
  ViewportProjectionFinder() = default;
  virtual ~ViewportProjectionFinder() = default;

  virtual std::pair<bool, Ogre::Vector3> getViewportPointProjectionOnXYPlane(
    RenderWindow * render_window, int x, int y);

private:
  virtual std::pair<bool, Ogre::Vector3> getViewportProjectionOnPlane(
    RenderWindow * render_window, int x, int y, Ogre::Plane & plane);
};

}  // namespace rviz_rendering

#endif  // RVIZ_RENDERING__VIEWPORT_PROJECTION_FINDER_HPP_
