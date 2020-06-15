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

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include <OgreEntity.h>
#include <OgreMesh.h>
#include <OgreManualObject.h>

#include "rviz_rendering/objects/arrow.hpp"
#include "rviz_rendering/objects/point_cloud.hpp"
#include "rviz_rendering/objects/shape.hpp"

#include "scene_graph_introspection.hpp"

namespace rviz_default_plugins
{

void assertArrowWithTransform(
  Ogre::SceneManager * scene_manager,
  Ogre::Vector3 position,
  Ogre::Vector3 scale,
  Ogre::Quaternion orientation)
{
  auto arrow_scene_node = findOneArrow(scene_manager->getRootSceneNode());
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

bool arrowIsVisible(Ogre::SceneNode * scene_node)
{
  auto arrow_head = findEntityByMeshName(
    scene_node, "rviz_cone.mesh");
  auto arrow_shaft = findEntityByMeshName(
    scene_node, "rviz_cylinder.mesh");

  return arrow_head->isVisible() && arrow_shaft->isVisible();
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

std::vector<Ogre::Entity *> findAllSpheres(Ogre::SceneNode * scene_node)
{
  return findAllEntitiesByMeshName(scene_node, "rviz_sphere.mesh");
}

std::vector<Ogre::Entity *> findAllCones(Ogre::SceneNode * scene_node)
{
  return findAllEntitiesByMeshName(scene_node, "rviz_cone.mesh");
}

std::vector<Ogre::Entity *> findAllCylinders(Ogre::SceneNode * scene_node)
{
  return findAllEntitiesByMeshName(scene_node, "rviz_cylinder.mesh");
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

Ogre::ManualObject * findOneManualObject(Ogre::SceneNode * scene_node)
{
  auto objects = findAllOgreObjectByType<Ogre::ManualObject>(scene_node, "ManualObject");
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
        if (shaft_entity &&
          shaft_entity->getParentSceneNode()->getParentSceneNode()->getParentSceneNode() ==
          arrow_scene_node)
        {
          arrows.push_back(arrow_scene_node);
        }
      }
    }
  }
  return arrows;
}

Ogre::SceneNode * findOneArrow(Ogre::SceneNode * scene_node)
{
  auto arrows = findAllArrows(scene_node);
  return arrows.empty() ? nullptr : arrows[0];
}

std::vector<Ogre::SceneNode *> findAllAxes(Ogre::SceneNode * scene_node)
{
  std::vector<Ogre::SceneNode *> axes;
  auto cylinder_entities = findAllEntitiesByMeshName(scene_node, "rviz_cylinder.mesh");
  if (!cylinder_entities.empty()) {
    for (auto const & cylinder_entity : cylinder_entities) {
      auto axes_scene_node = cylinder_entity
        ->getParentSceneNode()  // OffsetNode from cylinder_entity
        ->getParentSceneNode()  // SceneNode from cylinder_entity
        ->getParentSceneNode();  // SceneNode from axes
      if (axes_scene_node) {
        auto local_cylinder_entities = findAllEntitiesByMeshName(
          axes_scene_node, "rviz_cylinder.mesh");
        if (local_cylinder_entities.size() == 3 &&
          std::find(axes.begin(), axes.end(), axes_scene_node) == axes.end())
        {
          axes.push_back(axes_scene_node);
        }
      }
    }
  }
  return axes;
}

Ogre::SceneNode * findOneAxes(Ogre::SceneNode * scene_node)
{
  auto axes = findAllAxes(scene_node);
  return axes.empty() ? nullptr : axes[0];
}

std::vector<Ogre::Vector3> getPositionsFromNodes(const std::vector<Ogre::SceneNode *> & nodes)
{
  std::vector<Ogre::Vector3> positions(nodes.size(), Ogre::Vector3::ZERO);
  std::transform(
    nodes.cbegin(), nodes.cend(), positions.begin(), [](auto node) {
      return node->getPosition();
    });
  return positions;
}

std::vector<Ogre::Quaternion> getOrientationsFromNodes(const std::vector<Ogre::SceneNode *> & nodes)
{
  std::vector<Ogre::Quaternion> orientations(nodes.size(), Ogre::Quaternion::IDENTITY);
  std::transform(
    nodes.cbegin(), nodes.cend(), orientations.begin(), [](auto node) {
      return node->getOrientation();
    });
  return orientations;
}

}  // namespace rviz_default_plugins
