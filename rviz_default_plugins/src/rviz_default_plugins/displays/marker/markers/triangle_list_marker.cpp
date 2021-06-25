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

#include "rviz_default_plugins/displays/marker/markers/triangle_list_marker.hpp"

#include <iostream>
#include <fstream>
#include <vector>

#include <OgreDataStream.h>
#include <OgreImage.h>
#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreResourceGroupManager.h>
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreTextureManager.h>
#include <OgreTechnique.h>

#include "resource_retriever/retriever.hpp"
#include "rviz_rendering/mesh_loader.hpp"
#include "rviz_rendering/material_manager.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/load_resource.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/uniform_string_stream.hpp"

#include "rviz_default_plugins/displays/marker/markers/marker_selection_handler.hpp"
#include "rviz_default_plugins/displays/marker/marker_common.hpp"

namespace rviz_default_plugins
{
namespace displays
{
namespace markers
{

resource_retriever::MemoryResource getResource(const std::string & resource_path)
{
  resource_retriever::Retriever retriever;
  resource_retriever::MemoryResource res;
  try {
    res = retriever.get(resource_path);
  } catch (resource_retriever::Exception & e) {
    RVIZ_COMMON_LOG_DEBUG(e.what());
    return resource_retriever::MemoryResource();
  }

  return res;
}

bool loadImage(const Ogre::String& texture_name, const Ogre::String& texture_path)
{
  bool image_loaded = false;
  resource_retriever::MemoryResource res = getResource(texture_path);
  Ogre::String tex_ext;
  Ogre::String::size_type index_of_extension = texture_path.find_last_of('.');
  if (index_of_extension != Ogre::String::npos)
  {
    tex_ext = texture_path.substr(index_of_extension+1);
    Ogre::DataStreamPtr data_stream(new Ogre::MemoryDataStream(res.data.get(), res.size, false));
    Ogre::Image img;
    img.load(data_stream, tex_ext);
    Ogre::TextureManager::getSingleton().loadImage(texture_name,
      "rviz_rendering", img, Ogre::TEX_TYPE_2D, 0, 1.0f);
    image_loaded = true;
  }
  return image_loaded;
}

TriangleListMarker::TriangleListMarker(
  MarkerCommon * owner, rviz_common::DisplayContext * context, Ogre::SceneNode * parent_node)
: MarkerBase(owner, context, parent_node),
  manual_object_(nullptr)
{
}

TriangleListMarker::~TriangleListMarker()
{
  if (manual_object_) {
    context_->getSceneManager()->destroyManualObject(manual_object_);
    material_->unload();
    Ogre::MaterialManager::getSingleton().remove(material_);
  }
}

void TriangleListMarker::onNewMessage(
  const MarkerConstSharedPtr & old_message, const MarkerConstSharedPtr & new_message)
{
  assert(new_message->type == visualization_msgs::msg::Marker::TRIANGLE_LIST);

  if (wrongNumberOfPoints(new_message)) {
    printWrongNumberOfPointsError(new_message->points.size());
    scene_node_->setVisible(false);
    return;
  }

  if (!manual_object_) {
    initializeManualObject(new_message);
  }

  Ogre::Vector3 pos, scale;
  Ogre::Quaternion orient;
  if (!transform(new_message, pos, orient, scale)) {  // NOLINT: is super class method
    scene_node_->setVisible(false);
    return;
  }
  scene_node_->setVisible(true);

  if (owner_ && (new_message->scale.x * new_message->scale.y * new_message->scale.z == 0.0f) ) {
    owner_->setMarkerStatus(
      getID(), rviz_common::properties::StatusProperty::Warn, "Scale of 0 in one of x/y/z");
  }

  setPosition(pos);
  setOrientation(orient);
  scene_node_->setScale(scale);

  updateManualObject(old_message, new_message);

  handler_->addTrackedObject(manual_object_);
}

bool TriangleListMarker::wrongNumberOfPoints(const MarkerConstSharedPtr & new_message)
{
  size_t num_points = new_message->points.size();
  return (num_points % 3) != 0 || num_points == 0;
}

void TriangleListMarker::printWrongNumberOfPointsError(size_t num_points)
{
  std::stringstream ss;
  if (num_points == 0) {
    ss << "TriMesh marker [" << getStringID() << "] has no points.";
  } else {
    ss << "TriMesh marker [" << getStringID() <<
      "] has a point count which is not divisible by 3 [" << num_points << "]";
  }

  if (owner_) {
    owner_->setMarkerStatus(getID(), rviz_common::properties::StatusProperty::Error, ss.str());
  }

  RVIZ_COMMON_LOG_DEBUG(ss.str());
}

void TriangleListMarker::initializeManualObject(
  const MarkerBase::MarkerConstSharedPtr & new_message)
{
  static uint32_t count = 0;
  rviz_common::UniformStringStream ss;
  ss << "Triangle List Marker" << count++;
  manual_object_ = context_->getSceneManager()->createManualObject(ss.str());
  scene_node_->attachObject(manual_object_);

  ss << "Material";
  material_name_ = ss.str();
  material_ = rviz_rendering::MaterialManager::createMaterialWithLighting(material_name_);
  material_->setCullingMode(Ogre::CULL_NONE);
  handler_ = rviz_common::interaction::createSelectionHandler<MarkerSelectionHandler>(
    this, MarkerID(new_message->ns, new_message->id), context_);
}

void TriangleListMarker::updateManualObject(
  const MarkerBase::MarkerConstSharedPtr & old_message,
  const MarkerBase::MarkerConstSharedPtr & new_message) const
{
  beginManualObject(old_message, new_message);
  bool any_vertex_has_alpha = fillManualObjectAndDetermineAlpha(new_message);
  manual_object_->end();

  updateMaterial(new_message, any_vertex_has_alpha);
}

void TriangleListMarker::beginManualObject(
  const MarkerBase::MarkerConstSharedPtr & old_message,
  const MarkerBase::MarkerConstSharedPtr & new_message) const
{
  size_t num_points = new_message->points.size();

  // If we have the same number of tris as previously, just update the object
  if (old_message &&
    num_points == old_message->points.size() &&
    manual_object_->getNumSections() > 0)
  {
    manual_object_->beginUpdate(0);
  } else {  // Otherwise clear it and begin anew
    manual_object_->clear();
    manual_object_->estimateVertexCount(num_points);
    manual_object_->begin(
      material_name_, Ogre::RenderOperation::OT_TRIANGLE_LIST, "rviz_rendering");
  }
}

bool TriangleListMarker::fillManualObjectAndDetermineAlpha(
  const MarkerBase::MarkerConstSharedPtr new_message) const
{
  bool any_vertex_has_alpha = false;


  size_t num_points = new_message->points.size();
  const std::vector<geometry_msgs::msg::Point> & points = new_message->points;
  for (size_t i = 0; i < num_points; i += 3) {
    std::vector<Ogre::Vector3> corners(3);
    for (size_t c = 0; c < 3; c++) {
      corners[c] = Ogre::Vector3(
        static_cast<Ogre::Real>(points[i + c].x),
        static_cast<Ogre::Real>(points[i + c].y),
        static_cast<Ogre::Real>(points[i + c].z));
    }
    Ogre::Vector3 normal = (corners[1] - corners[0]).crossProduct(corners[2] - corners[0]);
    normal.normalise();

    for (size_t c = 0; c < 3; c++) {
      manual_object_->position(corners[c]);
      manual_object_->normal(normal);
      if (hasVertexColors(new_message)) {
        any_vertex_has_alpha = any_vertex_has_alpha ||
          (new_message->colors[i + c].a < rviz_rendering::unit_alpha_threshold);
        manual_object_->colour(
          new_message->colors[i + c].r,
          new_message->colors[i + c].g,
          new_message->colors[i + c].b,
          new_message->color.a * new_message->colors[i + c].a);
      } else if (hasFaceColors(new_message)) {
        any_vertex_has_alpha = any_vertex_has_alpha ||
          (new_message->colors[i / 3].a < rviz_rendering::unit_alpha_threshold);
        manual_object_->colour(
          new_message->colors[i / 3].r,
          new_message->colors[i / 3].g,
          new_message->colors[i / 3].b,
          new_message->color.a * new_message->colors[i / 3].a);
      }

      if (hasTexture(new_message))
      {
        manual_object_->textureCoord(
          new_message->uv_coordinates[i + c].u,
          new_message->uv_coordinates[i + c].v);
      }
    }
  }
  return any_vertex_has_alpha;
}

void TriangleListMarker::updateMaterial(
  const MarkerBase::MarkerConstSharedPtr & new_message,
  bool any_vertex_has_alpha) const
{
  if (hasVertexColors(new_message) || hasFaceColors(new_message)) {
    material_->getTechnique(0)->setLightingEnabled(false);
  } else {
    material_->getTechnique(0)->setLightingEnabled(true);
    float r, g, b, a;
    r = new_message->color.r;
    g = new_message->color.g;
    b = new_message->color.b;
    a = new_message->color.a;
    material_->getTechnique(0)->setAmbient(r / 2, g / 2, b / 2);
    material_->getTechnique(0)->setDiffuse(r, g, b, a);
  }

  if ((!hasVertexColors(new_message) &&
    new_message->color.a < rviz_rendering::unit_alpha_threshold) ||
    (hasVertexColors(new_message) && any_vertex_has_alpha))
  {
    material_->getTechnique(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    material_->getTechnique(0)->setDepthWriteEnabled(false);
  } else {
    material_->getTechnique(0)->setSceneBlending(Ogre::SBT_REPLACE);
    material_->getTechnique(0)->setDepthWriteEnabled(true);
  }

  if (hasTexture(new_message))
  {
    // If the texture is marked as updated, delete the last texture (if it exists).
    Ogre::ResourcePtr texture = Ogre::TextureManager::getSingleton().getByName(new_message->texture_map, "rviz_rendering");
    if (new_message->texture_update)
    {
      if (texture != NULL)
      {
        Ogre::TextureManager::getSingleton().remove(texture);
      }
    }
    if (Ogre::TextureManager::getSingleton().getByName(new_message->texture_map, "rviz_rendering") == NULL)
    {
      loadImage(new_message->texture_map, new_message->texture_map);
    }
    material_->getTechnique(0)->getPass(0)->createTextureUnitState(new_message->texture_map);
    material_->getTechnique(0)->getPass(0)->setSceneBlending(
      Ogre::SBT_TRANSPARENT_ALPHA);
  }
}

bool TriangleListMarker::hasFaceColors(const MarkerBase::MarkerConstSharedPtr new_message) const
{
  return new_message->colors.size() == new_message->points.size() / 3;
}

bool TriangleListMarker::hasVertexColors(const MarkerBase::MarkerConstSharedPtr new_message) const
{
  return new_message->colors.size() == new_message->points.size();
}

bool TriangleListMarker::hasTexture(const MarkerBase::MarkerConstSharedPtr new_message) const
{
  return !new_message->texture_map.empty() && new_message->uv_coordinates.size() == new_message->points.size();
}

S_MaterialPtr TriangleListMarker::getMaterials()
{
  S_MaterialPtr materials;
  materials.insert(material_);
  return materials;
}

}  // namespace markers
}  // namespace displays
}  // namespace rviz_default_plugins
