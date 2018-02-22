/*
 * Copyright (c) 2010, Willow Garage, Inc.
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

#include "mesh_resource_marker.hpp"

#include <string>

#ifdef _WIN32
# pragma warning(push)
# pragma warning(disable : 4996)
#endif

#include <OgreEntity.h>
#include <OgreMaterialManager.h>
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreSharedPtr.h>
#include <OgreSubEntity.h>
#include <OgreTechnique.h>
#include <OgreTextureManager.h>

#ifdef _WIN32
# pragma warning(pop)
#endif

#include "../marker_display.hpp"
#include "marker_selection_handler.hpp"
#include "rviz_common/selection/selection_manager.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/properties/status_property.hpp"
#include "rviz_rendering/mesh_loader.hpp"

namespace rviz_default_plugins
{
namespace displays
{
namespace markers
{

MeshResourceMarker::MeshResourceMarker(
  MarkerDisplay * owner, rviz_common::DisplayContext * context, Ogre::SceneNode * parent_node)
: MarkerBase(owner, context, parent_node), entity_(0)
{}

MeshResourceMarker::~MeshResourceMarker()
{
  reset();
}

void MeshResourceMarker::reset()
{
  // destroy entity
  if (entity_) {
    context_->getSceneManager()->destroyEntity(entity_);
    entity_ = 0;
  }


  // destroy all the materials we've created
  S_MaterialPtr::iterator it;
  for (it = materials_.begin(); it != materials_.end(); it++) {
    Ogre::MaterialPtr material = *it;
    if (material) {
      material->unload();
      // TODO(botteroa-si): check that remove(name, resource_group) is called properly.
      Ogre::MaterialManager::getSingletonPtr()->remove(material->getName(), material->getGroup());
    }
  }
  materials_.clear();
}

void MeshResourceMarker::onNewMessage(
  const MarkerConstSharedPtr & old_message, const MarkerConstSharedPtr & new_message)
{
  assert(new_message->type == visualization_msgs::msg::Marker::MESH_RESOURCE);

  // flag indicating if the mesh material color needs to be updated
  bool update_color = false;

  scene_node_->setVisible(false);

  // Get the color information from the message
  float r = new_message->color.r;
  float g = new_message->color.g;
  float b = new_message->color.b;
  float a = new_message->color.a;

  Ogre::SceneBlendType blending;
  bool depth_write;

  if (a < 0.9998) {
    blending = Ogre::SBT_TRANSPARENT_ALPHA;
    depth_write = false;
  } else {
    blending = Ogre::SBT_REPLACE;
    depth_write = true;
  }


  if (!entity_ ||
    old_message->mesh_resource != new_message->mesh_resource ||
    old_message->mesh_use_embedded_materials != new_message->mesh_use_embedded_materials)
  {
    reset();

    if (new_message->mesh_resource.empty()) {
      return;
    }

    if (!rviz_rendering::loadMeshFromResource(new_message->mesh_resource)) {
      std::stringstream ss;
      ss << "Mesh resource marker [" << getStringID() << "] could not load [" <<
        new_message->mesh_resource << "]";
      if (owner_) {
        owner_->setMarkerStatus(getID(), rviz_common::properties::StatusProperty::Error, ss.str());
      }
      RVIZ_COMMON_LOG_DEBUG(ss.str().c_str());
      return;
    }

    static uint32_t count = 0;
    std::stringstream ss;
    ss << "mesh_resource_marker_" << count++;
    std::string id = ss.str();
    entity_ = context_->getSceneManager()->createEntity(id, new_message->mesh_resource);
    scene_node_->attachObject(entity_);

    // create a default material for any sub-entities which don't have their own.
    ss << "Material";

    // TODO(botteroa-si): check that "rviz_rendering" is the ROS_PACKAGE_NAME we want here.
    Ogre::MaterialPtr default_material =
      Ogre::MaterialManager::getSingleton().create(ss.str(), "rviz_rendering");
    default_material->setReceiveShadows(false);
    default_material->getTechnique(0)->setLightingEnabled(true);
    default_material->getTechnique(0)->setAmbient(0.5, 0.5, 0.5);

    materials_.insert(default_material);

    if (new_message->mesh_use_embedded_materials) {
      // make clones of all embedded materials so selection works correctly
      S_MaterialPtr materials = getMaterials();

      S_MaterialPtr::iterator it;
      for (it = materials.begin(); it != materials.end(); it++) {
        if ((*it)->getName() != "BaseWhiteNoLighting") {
          Ogre::MaterialPtr new_material = (*it)->clone(id + (*it)->getName());
          materials_.insert(new_material);
        }
      }

      // make sub-entities use cloned materials
      for (uint32_t i = 0; i < entity_->getNumSubEntities(); ++i) {
        std::string mat_name = entity_->getSubEntity(i)->getMaterialName();
        if (mat_name != "BaseWhiteNoLighting") {
          entity_->getSubEntity(i)->setMaterialName(id + mat_name);
        } else {
          // BaseWhiteNoLighting is the default material Ogre uses
          // when it sees a mesh with no material.  Here we replace
          // that with our default_material which gets colored with
          // new_message->color.
          entity_->getSubEntity(i)->setMaterial(default_material);
        }
      }
    } else {
      entity_->setMaterial(default_material);
    }

    handler_.reset(new MarkerSelectionHandler(this, MarkerID(new_message->ns, new_message->id),
      context_));
    handler_->addTrackedObject(entity_);
  } else {
    // underlying mesh resource has not changed but if the color has
    //  then we need to update the materials color
    if (new_message->mesh_use_embedded_materials == false &&
      (!old_message ||
      old_message->mesh_use_embedded_materials == true ||
      old_message->color.r != new_message->color.r ||
      old_message->color.g != new_message->color.g ||
      old_message->color.b != new_message->color.b ||
      old_message->color.a != new_message->color.a))
    {
      update_color = true;
    }
  }

  // update material color
  //  if the mesh_use_embedded_materials is true and color is non-zero
  //  then the color will be used to tint the embedded materials
  if (update_color) {
    if (new_message->mesh_use_embedded_materials && r == 0 && g == 0 && b == 0 && a == 0) {
      blending = Ogre::SBT_REPLACE;
      depth_write = true;
      r = 1; g = 1; b = 1; a = 1;
    }
    S_MaterialPtr::iterator material_it;
    for (material_it = materials_.begin(); material_it != materials_.end(); material_it++) {
      Ogre::Technique * technique = (*material_it)->getTechnique(0);
      technique->setAmbient(r * 0.5, g * 0.5, b * 0.5);
      technique->setDiffuse(r, g, b, a);
      technique->setSceneBlending(blending);
      technique->setDepthWriteEnabled(depth_write);
      technique->setLightingEnabled(true);
    }
  }

  Ogre::Vector3 pos, scale;
  Ogre::Quaternion orient;
  if (!transform(new_message, pos, orient, scale)) {  // NOLINT: is super class method
    RVIZ_COMMON_LOG_DEBUG("Unable to transform marker message");
    scene_node_->setVisible(false);
    return;
  }

  scene_node_->setVisible(true);
  setPosition(pos);
  setOrientation(orient);

  scene_node_->setScale(scale);
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
