/*
 * Copyright (c) 2018, Bosch Software Innovations GmbH.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef RVIZ_DEFAULT_PLUGINS__SCENE_GRAPH_INTROSPECTION_HELPER_HPP_
#define RVIZ_DEFAULT_PLUGINS__SCENE_GRAPH_INTROSPECTION_HELPER_HPP_

#include <gmock/gmock.h>

#include <functional>
#include <vector>

#include <OgreBillboardChain.h>
#include <OgreQuaternion.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreString.h>
#include <OgreVector3.h>

#include "rviz_rendering/objects/point_cloud.hpp"
#include "rviz_rendering/objects/movable_text.hpp"

MATCHER_P(Vector3Eq, expected, "") {
  return Ogre::Math::Abs(expected.x - arg.x) < 0.0001f &&
         Ogre::Math::Abs(expected.y - arg.y) < 0.0001f &&
         Ogre::Math::Abs(expected.z - arg.z) < 0.0001f;
}

MATCHER_P(QuaternionEq, expected, "") {
  return Ogre::Math::Abs(expected.x - arg.x) < 0.0001f &&
         Ogre::Math::Abs(expected.y - arg.y) < 0.0001f &&
         Ogre::Math::Abs(expected.z - arg.z) < 0.0001f &&
         Ogre::Math::Abs(expected.w - arg.w) < 0.0001f;
}

MATCHER_P(ColourValueEq, expected, "") {
  return Ogre::Math::Abs(expected.r - arg.r) < 0.0001f &&
         Ogre::Math::Abs(expected.g - arg.g) < 0.0001f &&
         Ogre::Math::Abs(expected.b - arg.b) < 0.0001f &&
         Ogre::Math::Abs(expected.a - arg.a) < 0.0001f;
}

namespace rviz_default_plugins
{
bool arrowIsVisible(Ogre::SceneNode * scene_node);

bool axesAreVisible(Ogre::SceneNode * scene_node);

void assertArrowWithTransform(
  Ogre::SceneManager * scene_manager,
  Ogre::Vector3 position,
  Ogre::Vector3 scale,
  Ogre::Quaternion orientation);
}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__SCENE_GRAPH_INTROSPECTION_HELPER_HPP_
