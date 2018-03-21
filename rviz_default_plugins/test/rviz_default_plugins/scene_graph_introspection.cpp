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

#include <gtest/gtest.h>
#include <gmock/gmock.h>

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
#include <OgreMovableObject.h>

#include "rviz_rendering/objects/arrow.hpp"
#include "rviz_rendering/objects/shape.hpp"

#include "scene_graph_introspection.hpp"

namespace rviz_default_plugins
{

bool quaternionNearlyEqual(Ogre::Quaternion expected, Ogre::Quaternion actual)
{
  return Ogre::Math::Abs(expected.x - actual.x) < 0.0001f &&
         Ogre::Math::Abs(expected.y - actual.y) < 0.0001f &&
         Ogre::Math::Abs(expected.z - actual.z) < 0.0001f &&
         Ogre::Math::Abs(expected.w - actual.w) < 0.0001f;
}

bool vector3NearlyEqual(Ogre::Vector3 expected, Ogre::Vector3 actual)
{
  return Ogre::Math::Abs(expected.x - actual.x) < 0.0001f &&
         Ogre::Math::Abs(expected.y - actual.y) < 0.0001f &&
         Ogre::Math::Abs(expected.z - actual.z) < 0.0001f;
}

bool arrowIsVisible(Ogre::SceneManager * scene_manager)
{
  auto arrow_head = rviz_default_plugins::findEntityByMeshName(
    scene_manager->getRootSceneNode(), "rviz_cone.mesh");
  auto arrow_shaft = rviz_default_plugins::findEntityByMeshName(
    scene_manager->getRootSceneNode(), "rviz_cylinder.mesh");

  return arrow_head->isVisible() && arrow_shaft->isVisible();
}

void assertArrowWithTransform(
  Ogre::SceneManager * scene_manager,
  Ogre::Vector3 position,
  Ogre::Vector3 scale,
  Ogre::Quaternion orientation)
{
  auto arrow_scene_node = rviz_default_plugins::findOneArrow(scene_manager->getRootSceneNode());
  ASSERT_TRUE(arrow_scene_node);
  EXPECT_VECTOR3_EQ(position, arrow_scene_node->getPosition());
  // Have to mangle the scale because of the default orientation of the cylinders (see arrow.cpp).
  EXPECT_VECTOR3_EQ(Ogre::Vector3(scale.z, scale.x, scale.y), arrow_scene_node->getScale());
  EXPECT_QUATERNION_EQ(orientation, arrow_scene_node->getOrientation());
}

std::vector<Ogre::Entity *> findAllEntitiesByMeshName(
  Ogre::SceneNode * scene_node, const Ogre::String & resource_name)
{
  std::vector<Ogre::Entity *> all_entities =
    findAllOgreObjectByType<Ogre::Entity>(scene_node, "Entity");

  std::vector<Ogre::Entity *> correct_entities;
  for (const auto & entity : all_entities) {
    if (entity->getMesh() && entity->getMesh()->getName() == resource_name) {
      correct_entities.push_back(entity);
    }
  }

  return correct_entities;
}

Ogre::Entity * findEntityByMeshName(
  Ogre::SceneNode * scene_node, const Ogre::String & resource_name)
{
  auto entities = findAllEntitiesByMeshName(scene_node, resource_name);
  return entities.empty() ? nullptr : entities[0];
}

Ogre::BillboardChain * findOneBillboardChain(Ogre::SceneNode * scene_node)
{
  auto objects = findAllOgreObjectByType<Ogre::BillboardChain>(scene_node, "BillboardChain");
  return objects.empty() ? nullptr : objects[0];
}
rviz_rendering::MovableText * findOneMovableText(Ogre::SceneNode * scene_node)
{
  auto objects = findAllOgreObjectByType<rviz_rendering::MovableText>(scene_node, "MovableText");
  return objects.empty() ? nullptr : objects[0];
}

Ogre::MovableObject * findOneMovableObject(Ogre::SceneNode * scene_node)
{
  auto objects = findAllOgreObjectByType<Ogre::MovableObject>(scene_node, "ManualObject");
  return objects.empty() ? nullptr : objects[0];
}

rviz_rendering::PointCloud * findOnePointCloud(Ogre::SceneNode * scene_node)
{
  auto point_clouds = findAllPointClouds(scene_node);
  return point_clouds.empty() ? nullptr : point_clouds[0];
}

std::vector<rviz_rendering::PointCloud *> findAllPointClouds(Ogre::SceneNode * scene_node)
{
  return findAllOgreObjectByType<rviz_rendering::PointCloud>(scene_node, "PointCloud");
}

std::vector<Ogre::SceneNode *> findAllArrows(Ogre::SceneNode * scene_node)
{
  std::vector<Ogre::SceneNode *> arrows;
  auto head_entities = findAllEntitiesByMeshName(scene_node, "rviz_cone.mesh");
  if (!head_entities.empty()) {
    for (auto const & head_entity : head_entities) {
      auto arrow_scene_node = head_entity
        ->getParentSceneNode()  // OffsetNode from head_entity
        ->getParentSceneNode()  // SceneNode from head_entity
        ->getParentSceneNode();  // SceneNode from arrow
      if (arrow_scene_node) {
        auto shaft_entity = findEntityByMeshName(arrow_scene_node, "rviz_cylinder.mesh");
        if (shaft_entity->getParentSceneNode()->getParentSceneNode()->getParentSceneNode() ==
          arrow_scene_node)
        {
          arrows.push_back(arrow_scene_node);
        }
      }
    }
  }
  return arrows;
}

std::vector<Ogre::SceneNode *> findAllAxes(Ogre::SceneNode * scene_node)
{
  std::vector<Ogre::SceneNode *> axes;
  auto all_cylinders = findAllEntitiesByMeshName(scene_node, "rviz_cylinder.mesh");
  if (!all_cylinders.empty()) {
    for (size_t i = 0; i < all_cylinders.size(); i++) {
      auto first_axis_node = all_cylinders[i]
        ->getParentSceneNode()  // OffsetNode from shape
        ->getParentSceneNode()  // SceneNode from shape
        ->getParentSceneNode();  // SceneNode from axes
      if (first_axis_node) {
        for (size_t j = i + 1; j < all_cylinders.size(); j++) {
          auto second_axis_node = all_cylinders[j]
            ->getParentSceneNode()
            ->getParentSceneNode()
            ->getParentSceneNode();
          if (second_axis_node && second_axis_node == first_axis_node) {
            for (size_t k = j + 1; k < all_cylinders.size(); k++) {
              auto third_axis_node = all_cylinders[k]
                ->getParentSceneNode()
                ->getParentSceneNode()
                ->getParentSceneNode();
              if (third_axis_node && third_axis_node == first_axis_node) {
                axes.push_back(first_axis_node);
              }
            }
          }
        }
      }
    }
  }

  return axes;
}

Ogre::SceneNode * findOneArrow(Ogre::SceneNode * scene_node)
{
  auto arrows = findAllArrows(scene_node);
  return arrows.empty() ? nullptr : arrows[0];
}

}  // namespace rviz_default_plugins
