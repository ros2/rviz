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

#include <vector>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreTextureManager.h>
#include <OgreTechnique.h>

#include "rviz_rendering/mesh_loader.hpp"
#include "rviz_common/display_context.hpp"
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
  material_ = Ogre::MaterialManager::getSingleton().create(material_name_, "rviz_rendering");
  material_->setReceiveShadows(false);
  material_->getTechnique(0)->setLightingEnabled(true);
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
        any_vertex_has_alpha = any_vertex_has_alpha || (new_message->colors[i + c].a < 0.9998);
        manual_object_->colour(new_message->colors[i + c].r,
          new_message->colors[i + c].g,
          new_message->colors[i + c].b,
          new_message->color.a * new_message->colors[i + c].a);
      } else if (hasFaceColors(new_message)) {
        any_vertex_has_alpha = any_vertex_has_alpha || (new_message->colors[i / 3].a < 0.9998);
        manual_object_->colour(new_message->colors[i / 3].r,
          new_message->colors[i / 3].g,
          new_message->colors[i / 3].b,
          new_message->color.a * new_message->colors[i / 3].a);
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

  if ( (!hasVertexColors(new_message) && new_message->color.a < 0.9998) ||
    (hasVertexColors(new_message) && any_vertex_has_alpha))
  {
    material_->getTechnique(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    material_->getTechnique(0)->setDepthWriteEnabled(false);
  } else {
    material_->getTechnique(0)->setSceneBlending(Ogre::SBT_REPLACE);
    material_->getTechnique(0)->setDepthWriteEnabled(true);
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

S_MaterialPtr TriangleListMarker::getMaterials()
{
  S_MaterialPtr materials;
  materials.insert(material_);
  return materials;
}

}  // namespace markers
}  // namespace displays
}  // namespace rviz_default_plugins
