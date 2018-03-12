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

#ifndef RVIZ_DEFAULT_PLUGINS__SCENE_GRAPH_INTROSPECTION_HPP_
#define RVIZ_DEFAULT_PLUGINS__SCENE_GRAPH_INTROSPECTION_HPP_

#include <gtest/gtest.h>
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

#define EXPECT_QUATERNION_EQ(expected, actual) \
  EXPECT_PRED2(rviz_default_plugins::quaternionNearlyEqual, expected, actual)
#define ASSERT_QUATERNION_EQ(expected, actual) \
  ASSERT_PRED2(rviz_default_plugins::quaternionNearlyEqual, expected, actual)

#define EXPECT_VECTOR3_EQ(expected, actual) \
  EXPECT_PRED2(rviz_default_plugins::vector3NearlyEqual, expected, actual)
#define ASSERT_VECTOR3_EQ(expected, actual) \
  ASSERT_PRED2(rviz_default_plugins::vector3NearlyEqual, expected, actual)

namespace rviz_default_plugins
{
// Used in MACRO
bool quaternionNearlyEqual(Ogre::Quaternion expected, Ogre::Quaternion actual);
bool vector3NearlyEqual(Ogre::Vector3 expected, Ogre::Vector3 actual);

std::vector<Ogre::SceneNode *> findAllArrows(Ogre::SceneNode * scene_node);
Ogre::SceneNode * findOneArrow(Ogre::SceneNode * scene_node);

std::vector<Ogre::Entity *> findAllEntitiesByMeshName(
  Ogre::SceneNode * scene_node, const Ogre::String & resource_name);
Ogre::Entity * findEntityByMeshName(
  Ogre::SceneNode * scene_node, const Ogre::String & resource_name);

std::vector<rviz_rendering::PointCloud *> findAllPointClouds(Ogre::SceneNode * scene_node);
rviz_rendering::PointCloud * findOnePointCloud(Ogre::SceneNode * scene_node);

Ogre::BillboardChain * findOneBillboardChain(Ogre::SceneNode * scene_node);

rviz_rendering::MovableText * findOneMovableText(Ogre::SceneNode * scene_node);

Ogre::MovableObject * findOneMovableObject(Ogre::SceneNode * scene_node);

template<typename OgreType>
void findAllObjectsAttached(
  Ogre::SceneNode * scene_node, Ogre::String type, std::vector<OgreType *> & objects)
{
  auto it = scene_node->getAttachedObjectIterator();
  while (it.hasMoreElements()) {
    auto movable_object = it.getNext();
    if (movable_object->getMovableType() == type) {
      auto entity = dynamic_cast<OgreType *>(movable_object);
      if (entity) {
        objects.push_back(entity);
      }
    }
  }
}

template<typename OgreType>
std::vector<OgreType *> findAllOgreObjectByType(Ogre::SceneNode * scene_node, Ogre::String type)
{
  std::vector<OgreType *> objects_vector;
  findAllObjectsAttached(scene_node, type, objects_vector);
  auto child_it = scene_node->getChildIterator();
  while (child_it.hasMoreElements()) {
    auto child_node = child_it.getNext();
    auto child_scene_node = dynamic_cast<Ogre::SceneNode *>(child_node);
    if (child_scene_node != nullptr) {
      auto child_objects_vector = findAllOgreObjectByType<OgreType>(child_scene_node, type);
      objects_vector.insert(
        objects_vector.cend(), child_objects_vector.begin(), child_objects_vector.end());
    }
  }

  return objects_vector;
}

}  // namespace rviz_default_plugins


#endif  // RVIZ_DEFAULT_PLUGINS__SCENE_GRAPH_INTROSPECTION_HPP_
