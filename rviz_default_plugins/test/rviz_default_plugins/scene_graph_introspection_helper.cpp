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

#include <gmock/gmock.h>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#ifdef _WIN32
#pragma warning(push)
#pragma warning(disable : 4996)
#include <OgreEntity.h>
#pragma warning(pop)
#else
#include <OgreEntity.h>
#endif
#include <OgreMesh.h>
#include <OgreManualObject.h>

#include "rviz_rendering/objects/arrow.hpp"
#include "rviz_rendering/objects/point_cloud.hpp"
#include "rviz_rendering/objects/shape.hpp"
#include "test/rviz_rendering/scene_graph_introspection.hpp"

#include "scene_graph_introspection_helper.hpp"

namespace rviz_default_plugins
{

bool arrowIsVisible(Ogre::SceneNode * scene_node)
{
  auto arrow_head = rviz_rendering::findEntityByMeshName(scene_node, "rviz_cone.mesh");
  auto arrow_shaft = rviz_rendering::findEntityByMeshName(scene_node, "rviz_cylinder.mesh");

  return arrow_head->isVisible() && arrow_shaft->isVisible();
}

void assertArrowWithTransform(
  Ogre::SceneManager * scene_manager,
  Ogre::Vector3 position,
  Ogre::Vector3 scale,
  Ogre::Quaternion orientation)
{
  auto arrow_scene_node = rviz_rendering::findOneArrow(scene_manager->getRootSceneNode());
  ASSERT_TRUE(arrow_scene_node);
  EXPECT_THAT(arrow_scene_node->getPosition(), Vector3Eq(position));
  // Have to mangle the scale because of the default orientation of the cylinders (see arrow.cpp).
  EXPECT_THAT(arrow_scene_node->getScale(), Vector3Eq(Ogre::Vector3(scale.z, scale.x, scale.y)));
  EXPECT_THAT(arrow_scene_node->getOrientation(), QuaternionEq(orientation));
}

bool axesAreVisible(Ogre::SceneNode * scene_node)
{
  bool axes_are_visible = true;
  for (uint16_t i = 0; i < 3; ++i) {
    auto child_node = dynamic_cast<Ogre::SceneNode *>(scene_node->getChild(i)->getChild(0));
    auto entity = dynamic_cast<Ogre::Entity *>(child_node->getAttachedObject(0));
    axes_are_visible = entity ? axes_are_visible && entity->isVisible() : false;
  }

  return axes_are_visible;
}

}  // namespace rviz_default_plugins
