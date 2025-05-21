// Copyright (c) 2024, Open Source Robotics Foundation, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#ifndef RVIZ_RENDERING__OBJECTS__TRIANGLE_POLYGON_HPP_
#define RVIZ_RENDERING__OBJECTS__TRIANGLE_POLYGON_HPP_

#include <string>

#include <OgreColourValue.h>
#include <OgreManualObject.h>
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreVector.h>

#include "rviz_rendering/visibility_control.hpp"

namespace rviz_rendering
{
class TrianglePolygon
{
public:
  RVIZ_RENDERING_PUBLIC
  TrianglePolygon(
    Ogre::SceneManager * manager,
    Ogre::SceneNode * node,
    const Ogre::Vector3 & O,
    const Ogre::Vector3 & A,
    const Ogre::Vector3 & B,
    const std::string & name,
    const Ogre::ColourValue & color,
    bool use_color,
    bool upper_triangle);

  RVIZ_RENDERING_PUBLIC
  virtual ~TrianglePolygon();

  RVIZ_RENDERING_PUBLIC
  Ogre::ManualObject * getManualObject();

protected:
  Ogre::ManualObject * manual_;
  Ogre::SceneManager * manager_;
};

}  // namespace rviz_rendering
#endif  // RVIZ_RENDERING__OBJECTS__TRIANGLE_POLYGON_HPP_
