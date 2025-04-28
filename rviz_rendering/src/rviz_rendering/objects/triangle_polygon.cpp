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


#include "rviz_rendering/objects/triangle_polygon.hpp"

#include <string>

#include <OgreColourValue.h>
#include <OgreManualObject.h>
#include <OgreRenderOperation.h>
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreVector.h>

namespace rviz_rendering
{
TrianglePolygon::TrianglePolygon(
  Ogre::SceneManager * manager,
  Ogre::SceneNode * node,
  const Ogre::Vector3 & O,
  const Ogre::Vector3 & A,
  const Ogre::Vector3 & B,
  const std::string & name,
  const Ogre::ColourValue & color,
  bool use_color,
  bool upper_triangle)
{
  if (!manager || !node) {
    throw std::invalid_argument("SceneManager and SceneNode must not be null.");
  }

  // uniq string is requred for name
  manual_ = manager->createManualObject();
  manual_->clear();
  manual_->begin(
    name,
    Ogre::RenderOperation::OT_TRIANGLE_STRIP);
  manual_->position(O.x, O.y, O.z);
  if (upper_triangle) {
    manual_->textureCoord(0, 0);
  } else {
    manual_->textureCoord(1, 0);
  }
  if (use_color) {
    manual_->colour(color);
  }
  manual_->position(A.x, A.y, A.z);
  if (upper_triangle) {
    manual_->textureCoord(1, 0);
  } else {
    manual_->textureCoord(1, 1);
  }
  if (use_color) {
    manual_->colour(color);
  }
  manual_->position(B.x, B.y, B.z);
  if (upper_triangle) {
    manual_->textureCoord(0, 1);
  } else {
    manual_->textureCoord(0, 1);
  }
  if (use_color) {
    manual_->colour(color);
  }
  manual_->end();
  node->attachObject(manual_);
}

TrianglePolygon::~TrianglePolygon()
{
  manual_->detachFromParent();
}

Ogre::ManualObject * TrianglePolygon::getManualObject()
{
  return manual_;
}
}  // namespace rviz_rendering
