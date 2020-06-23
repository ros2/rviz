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

#include <memory>
#include <string>

#include <OgreEntity.h>
#include <OgreMaterialManager.h>
#include <OgreMesh.h>
#include <OgreSubEntity.h>
#include <OgreTechnique.h>

#include "visualization_msgs/msg/marker.hpp"

#include "rviz_default_plugins/displays/marker/markers/mesh_resource_marker.hpp"

#include "../../../scene_graph_introspection.hpp"
#include "markers_test_fixture.hpp"
#include "../marker_messages.hpp"

using namespace ::testing;  // NOLINT

TEST_F(MarkersTestFixture, setMessage_with_no_transform_makes_node_invisible) {
  marker_ =
    makeMarker<rviz_default_plugins::displays::markers::MeshResourceMarker>();

  EXPECT_CALL(*frame_manager_, transform(_, _, _, _, _)).WillOnce(Return(false));  // NOLINT
  auto message = createDefaultMessage(visualization_msgs::msg::Marker::MESH_RESOURCE);

  marker_->setMessage(message);

  auto entity = rviz_default_plugins::findEntityByMeshName(
    scene_manager_->getRootSceneNode(), message.mesh_resource);
  ASSERT_TRUE(entity);
  EXPECT_FALSE(entity->isVisible());
}

TEST_F(MarkersTestFixture, setMessage_with_transform_sets_position_and_orientation) {
  marker_ =
    makeMarker<rviz_default_plugins::displays::markers::MeshResourceMarker>();
  mockValidTransform();
  auto message = createDefaultMessage(visualization_msgs::msg::Marker::MESH_RESOURCE);

  marker_->setMessage(message);

  auto entity = rviz_default_plugins::findEntityByMeshName(
    scene_manager_->getRootSceneNode(), message.mesh_resource);
  ASSERT_TRUE(entity);
  EXPECT_TRUE(entity->isVisible());
  EXPECT_THAT(entity->getParentSceneNode()->getPosition(), Vector3Eq(Ogre::Vector3(0, 1, 0)));
  EXPECT_THAT(
    entity->getParentSceneNode()->getOrientation(), QuaternionEq(Ogre::Quaternion(0, 0, 1, 0)));
}

TEST_F(MarkersTestFixture, setMessage_does_not_attach_entity_when_mesh_is_missing) {
  marker_ =
    makeMarker<rviz_default_plugins::displays::markers::MeshResourceMarker>();
  mockValidTransform();

  auto message = createDefaultMessage(visualization_msgs::msg::Marker::MESH_RESOURCE);
  message.mesh_resource = "package://missing_resource.dae";

  marker_->setMessage(message);

  auto entity = rviz_default_plugins::findEntityByMeshName(
    scene_manager_->getRootSceneNode(), message.mesh_resource);
  ASSERT_FALSE(entity);
}

TEST_F(MarkersTestFixture, setMessage_does_not_attache_entity_when_no_mesh_attached) {
  marker_ =
    makeMarker<rviz_default_plugins::displays::markers::MeshResourceMarker>();
  mockValidTransform();

  auto message = createDefaultMessage(visualization_msgs::msg::Marker::MESH_RESOURCE);
  message.mesh_resource = "";

  marker_->setMessage(message);

  auto entity = rviz_default_plugins::findEntityByMeshName(
    scene_manager_->getRootSceneNode(), message.mesh_resource);
  ASSERT_FALSE(entity);
}

TEST_F(MarkersTestFixture, setMessage_attaches_default_material_to_correct_mesh) {
  marker_ =
    makeMarker<rviz_default_plugins::displays::markers::MeshResourceMarker>();
  mockValidTransform();

  auto message = createDefaultMessage(visualization_msgs::msg::Marker::MESH_RESOURCE);
  message.mesh_use_embedded_materials = false;

  marker_->setMessage(message);

  auto entity = rviz_default_plugins::findEntityByMeshName(
    scene_manager_->getRootSceneNode(), message.mesh_resource);
  ASSERT_TRUE(entity);
  EXPECT_THAT(entity->getMesh()->getName(), StrEq(message.mesh_resource));
  EXPECT_THAT(
    entity->getSubEntity(0)->getMaterial()->getName(),
    MatchesRegex("mesh_resource_marker_.Material"));
}

TEST_F(MarkersTestFixture, setMessage_initially_sets_color_correctly) {
  marker_ =
    makeMarker<rviz_default_plugins::displays::markers::MeshResourceMarker>();
  mockValidTransform();

  auto message = createDefaultMessage(visualization_msgs::msg::Marker::MESH_RESOURCE);
  message.mesh_use_embedded_materials = false;

  marker_->setMessage(message);

  auto entity = rviz_default_plugins::findEntityByMeshName(
    scene_manager_->getRootSceneNode(), message.mesh_resource);
  ASSERT_TRUE(entity);
  EXPECT_THAT(
    entity->getSubEntity(0)->getMaterial()->getTechnique(0)->getPass(0)->getDiffuse(),
    Eq(Ogre::ColourValue(0, 1, 1, 1)));
}

TEST_F(MarkersTestFixture, setMessage_changes_color_on_new_message_changing_color) {
  marker_ =
    makeMarker<rviz_default_plugins::displays::markers::MeshResourceMarker>();
  mockValidTransform();

  auto message = createDefaultMessage(visualization_msgs::msg::Marker::MESH_RESOURCE);
  message.mesh_use_embedded_materials = false;

  marker_->setMessage(message);
  message.color.b = 0.0f;
  marker_->setMessage(message);

  auto entity = rviz_default_plugins::findEntityByMeshName(
    scene_manager_->getRootSceneNode(), message.mesh_resource);
  ASSERT_TRUE(entity);
  EXPECT_THAT(
    entity->getSubEntity(0)->getMaterial()->getTechnique(0)->getPass(0)->getDiffuse(),
    Eq(Ogre::ColourValue(0, 1, 0, 1)));
}

TEST_F(MarkersTestFixture, setMessage_uses_cloned_materials_to_make_selection_work) {
  marker_ =
    makeMarker<rviz_default_plugins::displays::markers::MeshResourceMarker>();
  mockValidTransform();
  auto message = createDefaultMessage(visualization_msgs::msg::Marker::MESH_RESOURCE);

  marker_->setMessage(message);

  auto entity = rviz_default_plugins::findEntityByMeshName(
    scene_manager_->getRootSceneNode(), message.mesh_resource);
  ASSERT_TRUE(entity);
  auto entity_material_name = entity->getSubEntity(0)->getMaterialName();
  EXPECT_THAT(
    entity_material_name,
    MatchesRegex("mesh_resource_marker_." + message.mesh_resource + "Material."));
}

TEST_F(MarkersTestFixture, setMessage_with_new_object_clears_old_entities_and_materials) {
  marker_ =
    makeMarker<rviz_default_plugins::displays::markers::MeshResourceMarker>();
  mockValidTransform();
  auto message = createDefaultMessage(visualization_msgs::msg::Marker::MESH_RESOURCE);

  marker_->setMessage(message);

  // get everything that needs to be deleted later on
  auto entity = rviz_default_plugins::findEntityByMeshName(
    scene_manager_->getRootSceneNode(), message.mesh_resource);
  ASSERT_TRUE(entity);
  EXPECT_EQ(1u, marker_->getMaterials().size());
  auto material_name = entity->getSubEntity(0)->getMaterialName();
  EXPECT_TRUE(Ogre::MaterialManager::getSingletonPtr()->getByName(material_name, "rviz_rendering"));

  message.mesh_resource = "package://rviz_default_plugins/test_meshes/pr2-base_large.dae";
  marker_->setMessage(message);

  // entity and material are cleaned up
  auto deleted_entity = rviz_default_plugins::findEntityByMeshName(
    scene_manager_->getRootSceneNode(), "package://rviz_default_plugins/test_meshes/pr2-base.dae");
  ASSERT_FALSE(deleted_entity);
  EXPECT_FALSE(
    Ogre::MaterialManager::getSingletonPtr()->getByName(material_name, "rviz_rendering"));
}
