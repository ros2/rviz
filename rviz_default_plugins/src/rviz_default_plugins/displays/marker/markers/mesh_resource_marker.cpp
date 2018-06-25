/*
 * Copyright (c) 2010, Willow Garage, Inc.
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

#include "rviz_default_plugins/displays/marker/markers/mesh_resource_marker.hpp"

#include <string>

#ifdef _WIN32
# pragma warning(push)
# pragma warning(disable:4996)
#endif

#include <OgreEntity.h>
#include <OgreSubEntity.h>

#ifdef _WIN32
# pragma warning(pop)
#endif

#include <OgreMaterialManager.h>
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreTechnique.h>
#include <OgreTextureManager.h>

#include "rviz_rendering/mesh_loader.hpp"
#include "rviz_rendering/material_manager.hpp"
#include "rviz_common/display_context.hpp"

#include "rviz_default_plugins/displays/marker/marker_common.hpp"
#include "rviz_default_plugins/displays/marker/markers/marker_selection_handler.hpp"

namespace rviz_default_plugins
{
namespace displays
{
namespace markers
{

MeshResourceMarker::MeshResourceMarker(
  MarkerCommon * owner, rviz_common::DisplayContext * context, Ogre::SceneNode * parent_node)
: MarkerBase(owner, context, parent_node), entity_(nullptr)
{}

MeshResourceMarker::~MeshResourceMarker()
{
  reset();
}

void MeshResourceMarker::reset()
{
  destroyEntity();
  destroyMaterials();
  materials_.clear();
}

void MeshResourceMarker::destroyEntity()
{
  if (entity_) {
    scene_node_->detachObject(entity_);
    context_->getSceneManager()->destroyEntity(entity_);
    entity_ = nullptr;
  }
}

void MeshResourceMarker::destroyMaterials() const
{
  for (auto & material : materials_) {
    if (material) {
      material->unload();
      Ogre::MaterialManager::getSingletonPtr()->remove(material->getName(), material->getGroup());
    }
  }
}

void MeshResourceMarker::onNewMessage(
  const MarkerConstSharedPtr & old_message, const MarkerConstSharedPtr & new_message)
{
  assert(new_message->type == visualization_msgs::msg::Marker::MESH_RESOURCE);

  scene_node_->setVisible(false);

  if (!entity_ ||
    old_message->mesh_resource != new_message->mesh_resource ||
    old_message->mesh_use_embedded_materials != new_message->mesh_use_embedded_materials)
  {
    reset();

    if (new_message->mesh_resource.empty()) {
      return;
    }

    if (!rviz_rendering::loadMeshFromResource(new_message->mesh_resource)) {
      printMeshLoadingError(new_message);
      return;
    }

    createMeshWithMaterials(new_message);

    handler_ = rviz_common::interaction::createSelectionHandler<MarkerSelectionHandler>(
      this, MarkerID(new_message->ns, new_message->id), context_);
    handler_->addTrackedObject(entity_);
  } else {
    // underlying mesh resource has not changed but if the color has then we need to update the
    // materials color
    if (!(new_message->mesh_use_embedded_materials) &&
      (!old_message ||
      old_message->mesh_use_embedded_materials ||
      old_message->color.r != new_message->color.r ||
      old_message->color.g != new_message->color.g ||
      old_message->color.b != new_message->color.b ||
      old_message->color.a != new_message->color.a))
    {
      updateMaterialColor(new_message);
    }
  }

  Ogre::Vector3 pos, scale;
  Ogre::Quaternion orient;
  if (!transform(new_message, pos, orient, scale)) {  // NOLINT: is super class method
    scene_node_->setVisible(false);
    return;
  }

  scene_node_->setVisible(true);
  setPosition(pos);
  setOrientation(orient);

  scene_node_->setScale(scale);
}

void MeshResourceMarker::printMeshLoadingError(const MarkerBase::MarkerConstSharedPtr & new_message)
{
  std::string error = "Mesh resource marker [" + getStringID() + "] could not load [" +
    new_message->mesh_resource + "]";
  if (owner_) {
    owner_->setMarkerStatus(getID(), rviz_common::properties::StatusProperty::Error, error);
  }
  RVIZ_COMMON_LOG_DEBUG(error);
}

void MeshResourceMarker::createMeshWithMaterials(
  const MarkerBase::MarkerConstSharedPtr & new_message)
{
  static uint32_t count = 0;
  std::string id = "mesh_resource_marker_" + std::to_string(count++);
  entity_ = context_->getSceneManager()->createEntity(id, new_message->mesh_resource);
  scene_node_->attachObject(entity_);

  // create a default material for any sub-entities which don't have their own.
  Ogre::MaterialPtr default_material = createDefaultMaterial(id + "Material");

  materials_.insert(default_material);

  if (new_message->mesh_use_embedded_materials) {
    cloneMaterials(id);  // make clones of all embedded materials so selection works correctly
    useClonedMaterials(id, default_material);
  } else {
    entity_->setMaterial(default_material);
  }

  updateMaterialColor(new_message);
}


Ogre::MaterialPtr MeshResourceMarker::createDefaultMaterial(const std::string & material_name) const
{
  Ogre::MaterialPtr default_material =
    rviz_rendering::MaterialManager::createMaterialWithLighting(material_name);
  default_material->getTechnique(0)->setAmbient(0.5, 0.5, 0.5);
  return default_material;
}

void MeshResourceMarker::useClonedMaterials(
  const std::string & id, const Ogre::MaterialPtr & default_material) const
{
  for (uint32_t i = 0; i < entity_->getNumSubEntities(); ++i) {
    std::string material_name = entity_->getSubEntity(i)->getMaterialName();
    if (material_name != "BaseWhiteNoLighting") {
      entity_->getSubEntity(i)->setMaterialName(id + material_name);
    } else {
      // BaseWhiteNoLighting is the default material Ogre uses
      // when it sees a mesh with no material.  Here we replace
      // that with our default_material which gets colored with
      // new_message->color.
      entity_->getSubEntity(i)->setMaterial(default_material);
    }
  }
}

void MeshResourceMarker::cloneMaterials(const std::string & id)
{
  for (auto const & material : getMaterials()) {
    if (material->getName() != "BaseWhiteNoLighting") {
      Ogre::MaterialPtr new_material = material->clone(id + material->getName());
      materials_.insert(new_material);
    }
  }
}

void
MeshResourceMarker::updateMaterialColor(const MarkerBase::MarkerConstSharedPtr & new_message) const
{
  //  if the mesh_use_embedded_materials is true and color is non-zero
  //  then the color will be used to tint the embedded materials
  float r = new_message->color.r;
  float g = new_message->color.g;
  float b = new_message->color.b;
  float a = new_message->color.a;

  Ogre::SceneBlendType blending;
  bool depth_write;

  rviz_rendering::MaterialManager::enableAlphaBlending(blending, depth_write, a);

  if (new_message->mesh_use_embedded_materials && r == 0 && g == 0 && b == 0 && a == 0) {
    blending = Ogre::SBT_REPLACE;
    depth_write = true;
    r = 1; g = 1; b = 1; a = 1;
  }

  for (auto & material : this->materials_) {
    Ogre::Technique * technique = material->getTechnique(0);
    technique->setAmbient(r * 0.5f, g * 0.5f, b * 0.5f);
    technique->setDiffuse(r, g, b, a);
    technique->setSceneBlending(blending);
    technique->setDepthWriteEnabled(depth_write);
    technique->setLightingEnabled(true);
  }
}

S_MaterialPtr MeshResourceMarker::getMaterials()
{
  S_MaterialPtr materials;
  if (entity_) {
    extractMaterials(entity_, materials);
  }
  return materials;
}

}  // namespace markers
}  // namespace displays
}  // namespace rviz_default_plugins
