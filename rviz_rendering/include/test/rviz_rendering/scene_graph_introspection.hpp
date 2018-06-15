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

#ifndef RVIZ_RENDERING__SCENE_GRAPH_INTROSPECTION_HPP_
#define RVIZ_RENDERING__SCENE_GRAPH_INTROSPECTION_HPP_

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
#include "rviz_rendering/visibility_control.hpp"

namespace rviz_rendering
{
RVIZ_RENDERING_PUBLIC
bool arrowIsVisible(Ogre::SceneManager * scene_manager);

RVIZ_RENDERING_PUBLIC
std::vector<Ogre::SceneNode *> findAllArrows(Ogre::SceneNode * scene_node);
RVIZ_RENDERING_PUBLIC
Ogre::SceneNode * findOneArrow(Ogre::SceneNode * scene_node);

RVIZ_RENDERING_PUBLIC
std::vector<Ogre::SceneNode *> findAllAxes(Ogre::SceneNode * scene_node);
RVIZ_RENDERING_PUBLIC
Ogre::SceneNode * findOneAxes(Ogre::SceneNode * scene_node);

RVIZ_RENDERING_PUBLIC
std::vector<Ogre::Vector3> getPositionsFromNodes(const std::vector<Ogre::SceneNode *> & nodes);
RVIZ_RENDERING_PUBLIC
std::vector<Ogre::Quaternion>
getOrientationsFromNodes(const std::vector<Ogre::SceneNode *> & nodes);

RVIZ_RENDERING_PUBLIC
std::vector<Ogre::Entity *> findAllEntitiesByMeshName(
  Ogre::SceneNode * scene_node, const Ogre::String & resource_name);
RVIZ_RENDERING_PUBLIC
Ogre::Entity * findEntityByMeshName(
  Ogre::SceneNode * scene_node, const Ogre::String & resource_name);

RVIZ_RENDERING_PUBLIC
std::vector<Ogre::Entity *> findAllSpheres(Ogre::SceneNode * scene_node);
RVIZ_RENDERING_PUBLIC
std::vector<Ogre::Entity *> findAllCones(Ogre::SceneNode * scene_node);
RVIZ_RENDERING_PUBLIC
std::vector<Ogre::Entity *> findAllCylinders(Ogre::SceneNode * scene_node);

RVIZ_RENDERING_PUBLIC
std::vector<rviz_rendering::PointCloud *> findAllPointClouds(Ogre::SceneNode * scene_node);
RVIZ_RENDERING_PUBLIC
rviz_rendering::PointCloud * findOnePointCloud(Ogre::SceneNode * scene_node);

RVIZ_RENDERING_PUBLIC
Ogre::BillboardChain * findOneBillboardChain(Ogre::SceneNode * scene_node);

RVIZ_RENDERING_PUBLIC
rviz_rendering::MovableText * findOneMovableText(Ogre::SceneNode * scene_node);

RVIZ_RENDERING_PUBLIC
Ogre::ManualObject * findOneManualObject(Ogre::SceneNode * scene_node);

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

}  // namespace rviz_rendering

#endif  // RVIZ_RENDERING__SCENE_GRAPH_INTROSPECTION_HPP_
