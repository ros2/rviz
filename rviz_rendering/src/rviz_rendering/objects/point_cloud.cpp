/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * Copyright (c) 2017, Bosch Software Innovations GmbH.
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

#include "rviz_rendering/objects/point_cloud.hpp"

#include <algorithm>
#include <cassert>
#include <sstream>
#include <vector>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreVector.h>
#include <OgreQuaternion.h>
#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreBillboardSet.h>
#include <OgreBillboard.h>
#include <OgreTexture.h>
#include <OgreTextureManager.h>
#include <OgreSharedPtr.h>
#include <OgreTechnique.h>
#include <OgreCamera.h>

#include "rviz_rendering/custom_parameter_indices.hpp"
#include "rviz_rendering/logging.hpp"
#include "rviz_rendering/material_manager.hpp"

// TODO(greimela): Add again after clearing up the module dependencies
// #include "rviz_rendering/selection/forwards.hpp"

#define VERTEX_BUFFER_CAPACITY (36 * 1024 * 10)

namespace rviz_rendering
{

static float g_point_vertices[3] =
{
  0.0f, 0.0f, 0.0f
};

static float g_billboard_vertices[6 * 3] =
{
  -0.5f, 0.5f, 0.0f,
  -0.5f, -0.5f, 0.0f,
  0.5f, 0.5f, 0.0f,
  0.5f, 0.5f, 0.0f,
  -0.5f, -0.5f, 0.0f,
  0.5f, -0.5f, 0.0f,
};

static float g_billboard_sphere_vertices[3 * 3] =
{
  0.0f, 1.0f, 0.0f,
  -0.866025404f, -0.5f, 0.0f,
  0.866025404f, -0.5f, 0.0f,
};

static float g_box_vertices[6 * 6 * 3] =
{
  // front
  -0.5f, 0.5f, -0.5f,
  -0.5f, -0.5f, -0.5f,
  0.5f, 0.5f, -0.5f,
  0.5f, 0.5f, -0.5f,
  -0.5f, -0.5f, -0.5f,
  0.5f, -0.5f, -0.5f,

  // back
  -0.5f, 0.5f, 0.5f,
  0.5f, 0.5f, 0.5f,
  -0.5f, -0.5f, 0.5f,
  0.5f, 0.5f, 0.5f,
  0.5f, -0.5f, 0.5f,
  -0.5f, -0.5f, 0.5f,

  // right
  0.5f, 0.5f, 0.5f,
  0.5f, 0.5f, -0.5f,
  0.5f, -0.5f, 0.5f,
  0.5f, 0.5f, -0.5f,
  0.5f, -0.5f, -0.5f,
  0.5f, -0.5f, 0.5f,

  // left
  -0.5f, 0.5f, 0.5f,
  -0.5f, -0.5f, 0.5f,
  -0.5f, 0.5f, -0.5f,
  -0.5f, 0.5f, -0.5f,
  -0.5f, -0.5f, 0.5f,
  -0.5f, -0.5f, -0.5f,

  // top
  -0.5f, 0.5f, -0.5f,
  0.5f, 0.5f, -0.5f,
  -0.5f, 0.5f, 0.5f,
  0.5f, 0.5f, -0.5f,
  0.5f, 0.5f, 0.5f,
  -0.5f, 0.5f, 0.5f,

  // bottom
  -0.5f, -0.5f, -0.5f,
  -0.5f, -0.5f, 0.5f,
  0.5f, -0.5f, -0.5f,
  0.5f, -0.5f, -0.5f,
  -0.5f, -0.5f, 0.5f,
  0.5f, -0.5f, 0.5f,
};

Ogre::String PointCloud::sm_Type = "PointCloud";

uint32_t PointCloud::getVerticesPerPoint()
{
  if (current_mode_supports_geometry_shader_) {
    return 1;
  }
  switch (render_mode_) {
    case RM_POINTS:
      return 1;
    case RM_SQUARES:
      return 6;
    case RM_FLAT_SQUARES:
      return 6;
    case RM_SPHERES:
      return 3;
    case RM_TILES:
      return 6;
    case RM_BOXES:
      return 36;
    default:
      throw std::runtime_error("unexpected render_mode_");
  }
}

float * PointCloud::getVertices()
{
  if (current_mode_supports_geometry_shader_) {
    return g_point_vertices;
  }
  switch (render_mode_) {
    case RM_POINTS:
      return g_point_vertices;
    case RM_SQUARES:
      return g_billboard_vertices;
    case RM_FLAT_SQUARES:
      return g_billboard_vertices;
    case RM_SPHERES:
      return g_billboard_sphere_vertices;
    case RM_TILES:
      return g_billboard_vertices;
    case RM_BOXES:
      return g_box_vertices;
    default:
      throw std::runtime_error("unexpected render_mode_");
  }
}

Ogre::MaterialPtr PointCloud::getMaterialForRenderMode(RenderMode mode)
{
  switch (mode) {
    case RM_POINTS:
      return Ogre::MaterialPtr(point_material_);
    case RM_SQUARES:
      return Ogre::MaterialPtr(square_material_);
    case RM_FLAT_SQUARES:
      return Ogre::MaterialPtr(flat_square_material_);
    case RM_SPHERES:
      return Ogre::MaterialPtr(sphere_material_);
    case RM_TILES:
      return Ogre::MaterialPtr(tile_material_);
    case RM_BOXES:
      return Ogre::MaterialPtr(box_material_);
    default:
      throw std::runtime_error("unexpected render_mode_");
  }
}

PointCloud::PointCloud()
: point_count_(0),
  common_direction_(Ogre::Vector3::NEGATIVE_UNIT_Z),
  common_up_vector_(Ogre::Vector3::UNIT_Y),
  color_by_index_(false),
  current_mode_supports_geometry_shader_(false)
{
  std::stringstream ss;
  static int count = 0;
  ss << "PointCloudMaterial" << count++;
  point_material_ = Ogre::MaterialManager::getSingleton().getByName("rviz/PointCloudPoint");
  square_material_ = Ogre::MaterialManager::getSingleton().getByName("rviz/PointCloudSquare");
  flat_square_material_ = Ogre::MaterialManager::getSingleton().getByName(
    "rviz/PointCloudFlatSquare");
  sphere_material_ = Ogre::MaterialManager::getSingleton().getByName("rviz/PointCloudSphere");
  tile_material_ = Ogre::MaterialManager::getSingleton().getByName("rviz/PointCloudTile");
  box_material_ = Ogre::MaterialManager::getSingleton().getByName("rviz/PointCloudBox");

  point_material_ = Ogre::MaterialPtr(point_material_)->clone(ss.str() + "Point");
  square_material_ = Ogre::MaterialPtr(square_material_)->clone(ss.str() + "Square");
  flat_square_material_ = Ogre::MaterialPtr(flat_square_material_)->clone(ss.str() + "FlatSquare");
  sphere_material_ = Ogre::MaterialPtr(sphere_material_)->clone(ss.str() + "Sphere");
  tile_material_ = Ogre::MaterialPtr(tile_material_)->clone(ss.str() + "Tiles");
  box_material_ = Ogre::MaterialPtr(box_material_)->clone(ss.str() + "Box");

  point_material_->load();
  square_material_->load();
  flat_square_material_->load();
  sphere_material_->load();
  tile_material_->load();
  box_material_->load();

  setAlpha(1.0f);
  setRenderMode(RM_SPHERES);
  setDimensions(0.01f, 0.01f, 0.01f);

  clear();
}

static void removeMaterial(Ogre::MaterialPtr & material)
{
  Ogre::ResourcePtr resource(material);
  Ogre::MaterialManager::getSingleton().remove(resource);
}

PointCloud::~PointCloud()
{
  clear();

  point_material_->unload();
  square_material_->unload();
  flat_square_material_->unload();
  sphere_material_->unload();
  tile_material_->unload();
  box_material_->unload();

  removeMaterial(point_material_);
  removeMaterial(square_material_);
  removeMaterial(flat_square_material_);
  removeMaterial(sphere_material_);
  removeMaterial(tile_material_);
  removeMaterial(box_material_);
}

const Ogre::AxisAlignedBox & PointCloud::getBoundingBox() const
{
  return bounding_box_;
}

float PointCloud::getBoundingRadius() const
{
  return bounding_box_.isNull() ?
         0.0f :
         Ogre::Math::Sqrt(
    std::max(
      bounding_box_.getMaximum().squaredLength(),
      bounding_box_.getMinimum().squaredLength()));
}

void PointCloud::getWorldTransforms(Ogre::Matrix4 * xform) const
{
  *xform = _getParentNodeFullTransform();
}

void PointCloud::clear()
{
  point_count_ = 0;
  bounding_box_.setNull();

  if (getParentSceneNode()) {
    for (auto const & renderable : renderables_) {
      getParentSceneNode()->detachObject(renderable.get());
    }
    getParentSceneNode()->needUpdate();
  }

  renderables_.clear();
}

void PointCloud::clearAndRemoveAllPoints()
{
  clear();
  points_.clear();
}

void PointCloud::regenerateAll()
{
  if (point_count_ == 0) {
    return;
  }

  V_Point points;
  points.swap(points_);

  clear();

  addPoints(points.begin(), points.end());
}

void PointCloud::setColorByIndex(bool set)
{
  color_by_index_ = set;
  regenerateAll();
}

void PointCloud::setHighlightColor(float r, float g, float b)
{
  Ogre::Vector4 highlight(r, g, b, 0.0f);

  for (auto & renderable : renderables_) {
    renderable->setCustomParameter(RVIZ_RENDERING_HIGHLIGHT_PARAMETER, highlight);
  }
}

void PointCloud::setRenderMode(RenderMode mode)
{
  render_mode_ = mode;

  current_material_ = getMaterialForRenderMode(mode);
  current_material_->load();

  if (changingGeometrySupportIsNecessary(current_material_)) {
    renderables_.clear();
  }

  for (auto & renderable : renderables_) {
    renderable->setMaterial(current_material_);
  }

  regenerateAll();
}

bool PointCloud::changingGeometrySupportIsNecessary(Ogre::MaterialPtr const material)
{
  bool geom_support_changed = false;
  Ogre::Technique * best = material->getBestTechnique();
  if (best) {
    if (material->getBestTechnique()->getName() == "gp") {
      if (!current_mode_supports_geometry_shader_) {
        geom_support_changed = true;
      }

      current_mode_supports_geometry_shader_ = true;
    } else {
      if (current_mode_supports_geometry_shader_) {
        geom_support_changed = true;
      }

      current_mode_supports_geometry_shader_ = false;
    }
  } else {
    geom_support_changed = true;
    current_mode_supports_geometry_shader_ = false;

    RVIZ_RENDERING_LOG_ERROR_STREAM(
      "No techniques available for material [" << material->getName().c_str() << "]");
  }
  return geom_support_changed;
}

void PointCloud::setDimensions(float width, float height, float depth)
{
  point_extensions_ = Ogre::Vector4(width, height, depth, 0.0f);

  for (auto & renderable : renderables_) {
    renderable->setCustomParameter(RVIZ_RENDERING_SIZE_PARAMETER, point_extensions_);
  }
}

void PointCloud::setAutoSize(bool auto_size)
{
  for (auto & renderable : renderables_) {
    renderable->setCustomParameter(RVIZ_RENDERING_AUTO_SIZE_PARAMETER, Ogre::Vector4(auto_size));
  }
}

void PointCloud::setCommonDirection(const Ogre::Vector3 & vec)
{
  common_direction_ = vec;

  for (auto & renderable : renderables_) {
    renderable->setCustomParameter(RVIZ_RENDERING_NORMAL_PARAMETER, Ogre::Vector4(vec));
  }
}

void PointCloud::setCommonUpVector(const Ogre::Vector3 & vec)
{
  common_up_vector_ = vec;
  for (auto & renderable : renderables_) {
    renderable->setCustomParameter(RVIZ_RENDERING_UP_PARAMETER, Ogre::Vector4(vec));
  }
}

void setAlphaBlending(const Ogre::MaterialPtr & mat)
{
  if (mat->getBestTechnique()) {
    mat->getBestTechnique()->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    mat->getBestTechnique()->setDepthWriteEnabled(false);
  }
}

void setReplace(const Ogre::MaterialPtr & mat)
{
  if (mat->getBestTechnique()) {
    mat->getBestTechnique()->setSceneBlending(Ogre::SBT_REPLACE);
    mat->getBestTechnique()->setDepthWriteEnabled(true);
  }
}

void PointCloud::setAlpha(float alpha, bool per_point_alpha)
{
  alpha_ = alpha;

  if (alpha < rviz_rendering::unit_alpha_threshold || per_point_alpha) {
    setAlphaBlending(point_material_);
    setAlphaBlending(square_material_);
    setAlphaBlending(flat_square_material_);
    setAlphaBlending(sphere_material_);
    setAlphaBlending(tile_material_);
    setAlphaBlending(box_material_);
  } else {
    setReplace(point_material_);
    setReplace(square_material_);
    setReplace(flat_square_material_);
    setReplace(sphere_material_);
    setReplace(tile_material_);
    setReplace(box_material_);
  }

  Ogre::Vector4 alpha4(alpha_, alpha_, alpha_, alpha_);
  for (auto & renderable : renderables_) {
    renderable->setCustomParameter(RVIZ_RENDERING_ALPHA_PARAMETER, alpha4);
  }
}

void PointCloud::setColor(const Ogre::ColourValue & color)
{
  for (auto & point : points_) {
    point.setColor(color.r, color.g, color.b, color.a);
  }
  regenerateAll();
}

void PointCloud::addPoints(
  std::vector<Point>::iterator start_iterator,
  std::vector<Point>::iterator stop_iterator)
{
  if (stop_iterator - start_iterator <= 0) {
    return;
  }
  auto num_points = static_cast<uint32_t>(std::distance(start_iterator, stop_iterator));
  points_.insert(points_.cend(), start_iterator, stop_iterator);

  RenderableInternals internals = createNewRenderable(num_points);

  for (auto current_point = start_iterator; current_point < stop_iterator; ++current_point) {
    if (internals.bufferIsFull()) {
      assert(internals.noBufferOverflowOccurred());

      finishRenderable(
        internals,
        static_cast<uint32_t>(internals.rend->getBuffer()->getNumVertices()));

      internals = createNewRenderable(static_cast<uint32_t>(stop_iterator - current_point));
    }
    internals.aabb.merge(current_point->position);
    internals = addPointToHardwareBuffer(
      internals, current_point,
      static_cast<uint32_t>(current_point - start_iterator));
  }

  finishRenderable(internals, internals.current_vertex_count);

  point_count_ += num_points;

  if (getParentSceneNode()) {
    getParentSceneNode()->needUpdate();
  }
}

PointCloud::RenderableInternals
PointCloud::createNewRenderable(uint32_t number_of_points_to_be_added)
{
  RenderableInternals internals;
  internals.buffer_size = std::min<uint32_t>(
    VERTEX_BUFFER_CAPACITY,
    number_of_points_to_be_added * getVerticesPerPoint());

  internals.rend = createRenderable(internals.buffer_size, getRenderOperationType());

  internals.float_buffer = reinterpret_cast<float *>(internals.rend->getBuffer()
    ->lock(Ogre::HardwareBuffer::HBL_NO_OVERWRITE));

  internals.aabb.setNull();
  return internals;
}

Ogre::RenderOperation::OperationType PointCloud::getRenderOperationType() const
{
  Ogre::RenderOperation::OperationType op_type;
  if (current_mode_supports_geometry_shader_) {
    op_type = Ogre::RenderOperation::OT_POINT_LIST;
  } else {
    if (render_mode_ == RM_POINTS) {
      op_type = Ogre::RenderOperation::OT_POINT_LIST;
    } else {
      op_type = Ogre::RenderOperation::OT_TRIANGLE_LIST;
    }
  }
  return op_type;
}

void
PointCloud::finishRenderable(
  PointCloud::RenderableInternals internals,
  uint32_t vertex_count_of_renderable)
{
  Ogre::RenderOperation * op = internals.rend->getRenderOperation();
  op->vertexData->vertexCount = vertex_count_of_renderable - op->vertexData->vertexStart;
  internals.rend->setBoundingBox(internals.aabb);
  bounding_box_.merge(internals.aabb);
  assert(
    op->vertexData->vertexCount + op->vertexData->vertexStart <=
    internals.rend->getBuffer()->getNumVertices());

  internals.rend->getBuffer()->unlock();
}

uint32_t PointCloud::getColorForPoint(
  uint32_t current_point,
  std::vector<PointCloud::Point>::iterator point) const
{
  uint32_t color;

  if (color_by_index_) {
    // convert to ColourValue, so we can then convert to the rendersystem-specific color type
    color = (current_point + point_count_ + 1);
    Ogre::ColourValue c;
    c.a = 1.0f;
    c.r = ((color >> 16) & 0xff) / 255.0f;
    c.g = ((color >> 8) & 0xff) / 255.0f;
    c.b = (color & 0xff) / 255.0f;
    color = c.getAsBYTE();
  } else {
    color = point->color.getAsBYTE();
  }
  return color;
}

PointCloud::RenderableInternals
PointCloud::addPointToHardwareBuffer(
  PointCloud::RenderableInternals internals,
  std::vector<PointCloud::Point>::iterator point, uint32_t current_point)
{
  uint32_t color = getColorForPoint(current_point, point);
  float * vertices = getVertices();
  float * float_buffer = internals.float_buffer;

  float x = point->position.x;
  float y = point->position.y;
  float z = point->position.z;

  for (uint32_t j = 0; j < getVerticesPerPoint(); ++j, ++internals.current_vertex_count) {
    *float_buffer++ = x;
    *float_buffer++ = y;
    *float_buffer++ = z;

    if (!current_mode_supports_geometry_shader_) {
      *float_buffer++ = vertices[(j * 3)];
      *float_buffer++ = vertices[(j * 3) + 1];
      *float_buffer++ = vertices[(j * 3) + 2];
    }

    auto iptr = reinterpret_cast<uint32_t *>(float_buffer);
    *iptr = color;
    ++float_buffer;
  }
#ifndef NDEBUG
  size_t num_vertices = internals.rend->getBuffer()->getNumVertices();
  size_t vertex_size =
    internals.rend->getRenderOperation()->vertexData->vertexDeclaration->getVertexSize(0);
  assert(
    reinterpret_cast<uint8_t *>(float_buffer) <=
    reinterpret_cast<uint8_t *>(internals.float_buffer) + num_vertices * vertex_size);
#endif

  internals.float_buffer = float_buffer;
  return internals;
}

void PointCloud::popPoints(uint32_t num_points)
{
  assert(num_points <= point_count_);

  points_.erase(points_.begin(), points_.begin() + num_points);
  point_count_ -= num_points;

  uint32_t vpp = getVerticesPerPoint();
  size_t popped_count = removePointsFromRenderables(num_points, vpp);
  (void) popped_count;

  assert(popped_count == num_points * vpp);

  resetBoundingBoxForCurrentPoints();

  if (getParentSceneNode()) {
    getParentSceneNode()->needUpdate();
  }
}

std::vector<PointCloud::Point> PointCloud::getPoints()
{
  return points_;
}

size_t PointCloud::removePointsFromRenderables(
  uint32_t number_of_points, uint32_t
  vertices_per_point)
{
  size_t popped_count = 0;
  while (popped_count < number_of_points * vertices_per_point) {
    PointCloudRenderablePtr rend = renderables_.front();
    Ogre::RenderOperation * op = rend->getRenderOperation();

    size_t popped_in_renderable = std::min(
      static_cast<size_t>(number_of_points * vertices_per_point - popped_count),
      op->vertexData->vertexCount);
    op->vertexData->vertexStart += popped_in_renderable;
    op->vertexData->vertexCount -= popped_in_renderable;

    popped_count += popped_in_renderable;

    if (op->vertexData->vertexCount == 0) {
      renderables_.pop_front();
    }
  }
  return popped_count;
}

void PointCloud::resetBoundingBoxForCurrentPoints()
{
  bounding_box_.setNull();
  for (uint32_t i = 0; i < point_count_; ++i) {
    Point & p = points_[i];
    bounding_box_.merge(p.position);
  }
}

void PointCloud::_notifyCurrentCamera(Ogre::Camera * camera)
{
  Ogre::MovableObject::_notifyCurrentCamera(camera);
}

void PointCloud::_updateRenderQueue(Ogre::RenderQueue * queue)
{
  for (auto & renderable : renderables_) {
    queue->addRenderable(renderable.get());
  }
}

void PointCloud::_notifyAttached(Ogre::Node * parent, bool isTagPoint)
{
  Ogre::MovableObject::_notifyAttached(parent, isTagPoint);
}

void PointCloud::setPickColor(const Ogre::ColourValue & color)
{
  pick_color_ = color;
  Ogre::Vector4 pick_col(pick_color_.r, pick_color_.g, pick_color_.b, pick_color_.a);

  for (auto & renderable : renderables_) {
    renderable->setCustomParameter(RVIZ_RENDERING_PICK_COLOR_PARAMETER, pick_col);
  }
  // TODO(greimela): Add again after clearing up the module dependencies
//  getUserObjectBindings().setUserAny( "pick_handle", Ogre::Any(
// selection::colorToHandle( color )));
}

PointCloudRenderablePtr PointCloud::createRenderable(
  int num_points,
  Ogre::RenderOperation::OperationType operation_type)
{
  PointCloudRenderablePtr rend(new PointCloudRenderable(
      this, num_points, !current_mode_supports_geometry_shader_, operation_type));
  rend->setMaterial(current_material_);
  Ogre::Vector4 alpha(alpha_, 0.0f, 0.0f, 0.0f);
  Ogre::Vector4 highlight(0.0f, 0.0f, 0.0f, 0.0f);
  Ogre::Vector4 pick_col(pick_color_.r, pick_color_.g, pick_color_.b, pick_color_.a);
  rend->setCustomParameter(RVIZ_RENDERING_SIZE_PARAMETER, Ogre::Vector4(point_extensions_));
  rend->setCustomParameter(RVIZ_RENDERING_ALPHA_PARAMETER, alpha);
  rend->setCustomParameter(RVIZ_RENDERING_HIGHLIGHT_PARAMETER, highlight);
  rend->setCustomParameter(RVIZ_RENDERING_PICK_COLOR_PARAMETER, pick_col);
  rend->setCustomParameter(RVIZ_RENDERING_NORMAL_PARAMETER, Ogre::Vector4(common_direction_));
  rend->setCustomParameter(RVIZ_RENDERING_UP_PARAMETER, Ogre::Vector4(common_up_vector_));
  if (getParentSceneNode()) {
    getParentSceneNode()->attachObject(rend.get());
  }
  renderables_.push_back(rend);

  return rend;
}

PointCloudRenderableQueue PointCloud::getRenderables()
{
  return renderables_;
}

void PointCloud::visitRenderables(Ogre::Renderable::Visitor * visitor, bool debugRenderables)
{
  (void) visitor;
  (void) debugRenderables;
}

}  // namespace rviz_rendering
