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

#ifndef RVIZ_RENDERING__OBJECTS__POINT_CLOUD_HPP_
#define RVIZ_RENDERING__OBJECTS__POINT_CLOUD_HPP_

#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include <OgreSimpleRenderable.h>
#include <OgreMovableObject.h>
#include <OgreString.h>
#include <OgreAxisAlignedBox.h>
#include <OgreVector.h>
#include <OgreMaterial.h>
#include <OgreColourValue.h>
#include <OgreRoot.h>
#include <OgreHardwareBufferManager.h>
#include <OgreSharedPtr.h>

#include "point_cloud_renderable.hpp"
#include "rviz_rendering/visibility_control.hpp"

namespace Ogre
{
class SceneManager;
class ManualObject;
class SceneNode;
class RenderQueue;
class Camera;
class RenderSystem;
class Matrix4;
}

namespace rviz_rendering
{

/**
 * \class PointCloud
 * \brief A visual representation of a set of points.
 *
 * Displays a set of points using any number of Ogre BillboardSets.
 * PointCloud is optimized for sets of points that change rapidly, rather than
 * for large clouds that never change.
 *
 * Most of the functions in PointCloud are not safe to call from any thread but
 * the render thread.
 * Exceptions are clear() and addPoints(), which are safe as long as we are not
 * in the middle of a render (ie. Ogre::Root::renderOneFrame, or
 * Ogre::RenderWindow::update).
 */
class PointCloud : public Ogre::MovableObject
{
public:
  enum RenderMode
  {
    RM_POINTS,
    RM_SQUARES,
    RM_FLAT_SQUARES,
    RM_SPHERES,
    RM_TILES,
    RM_BOXES,
  };

  RVIZ_RENDERING_PUBLIC
  PointCloud();

  RVIZ_RENDERING_PUBLIC
  virtual ~PointCloud();

  /**
   * \brief Clear all the points
   */
  RVIZ_RENDERING_PUBLIC
  void clear();

  /**
   * \brief calls clear() and removes all elements from points_
   */
  RVIZ_RENDERING_PUBLIC
  void clearAndRemoveAllPoints();

  /**
   * \struct Point
   * \brief Representation of a point, with x/y/z position and r/g/b color
   */
  struct Point
  {
    inline void setColor(float r, float g, float b, float a = 1.0)
    {
      color = Ogre::ColourValue(r, g, b, a);
    }

    Ogre::Vector3 position;
    Ogre::ColourValue color;
  };

  /**
   * \brief Add points to this point cloud
   *
   * \param start_iterator A std::vector::iterator to the start of the point vector to be added
   * \param end_iterator A std::vector::iterator to the end of the point vector to be added
   */
  RVIZ_RENDERING_PUBLIC
  void addPoints(
    std::vector<Point>::iterator start_iterator,
    std::vector<Point>::iterator end_iterator);

  /**
   * \brief Remove a number of points from this point cloud
   * \param num_points The number of points to pop
   */
  RVIZ_RENDERING_PUBLIC
  void popPoints(uint32_t num_points);

  RVIZ_RENDERING_PUBLIC
  std::vector<Point> getPoints();

  /// Set type of rendering primitive to used; supports points, billboards, spheres and boxes.
  RVIZ_RENDERING_PUBLIC
  void setRenderMode(RenderMode mode);

  /// Set the dimensions of the billboards used to render each point.
  /**
   * Width/height are only applicable to billboards and boxes, depth is only applicable to boxes.
   */
  RVIZ_RENDERING_PUBLIC
  void setDimensions(float width, float height, float depth);

  /**
   * \brief If set to true, the size of each point will be multiplied by its z component.
   * \param auto_size resize in shaders
   * @note (Used for depth image based point clouds)
   */
  RVIZ_RENDERING_PUBLIC
  void setAutoSize(bool auto_size);

  /// See Ogre::BillboardSet::setCommonDirection
  RVIZ_RENDERING_PUBLIC
  void setCommonDirection(const Ogre::Vector3 & vec);

  /// See Ogre::BillboardSet::setCommonUpVector
  RVIZ_RENDERING_PUBLIC
  void setCommonUpVector(const Ogre::Vector3 & vec);

  /**
   * \brief Set alpha blending
   * \param alpha global alpha value
   * \param per_point_alpha indicates that each point will have an individual alpha value.
   *   If true, enables alpha blending regardless of the global alpha.
   */
  RVIZ_RENDERING_PUBLIC
  void setAlpha(float alpha, bool per_point_alpha = false);

  RVIZ_RENDERING_PUBLIC
  void setColor(const Ogre::ColourValue & color);

  RVIZ_RENDERING_PUBLIC
  void setPickColor(const Ogre::ColourValue & color);

  RVIZ_RENDERING_PUBLIC
  void setColorByIndex(bool set);

  RVIZ_RENDERING_PUBLIC
  void setHighlightColor(float r, float g, float b);

  RVIZ_RENDERING_PUBLIC
  const Ogre::String & getMovableType() const override {return sm_Type;}

  RVIZ_RENDERING_PUBLIC
  const Ogre::AxisAlignedBox & getBoundingBox() const override;

  RVIZ_RENDERING_PUBLIC
  float getBoundingRadius() const override;

  RVIZ_RENDERING_PUBLIC
  virtual void getWorldTransforms(Ogre::Matrix4 * xform) const;

  RVIZ_RENDERING_PUBLIC
  virtual uint16_t getNumWorldTransforms() const {return 1;}

  RVIZ_RENDERING_PUBLIC
  void _updateRenderQueue(Ogre::RenderQueue * queue) override;

  RVIZ_RENDERING_PUBLIC
  void _notifyCurrentCamera(Ogre::Camera * camera) override;

  RVIZ_RENDERING_PUBLIC
  void _notifyAttached(Ogre::Node * parent, bool isTagPoint = false) override;

  RVIZ_RENDERING_PUBLIC
  void visitRenderables(Ogre::Renderable::Visitor * visitor, bool debugRenderables) override;

  RVIZ_RENDERING_PUBLIC
  virtual void setName(const std::string & name) {mName = name;}

  RVIZ_RENDERING_PUBLIC
  PointCloudRenderableQueue getRenderables();

  RVIZ_RENDERING_PUBLIC
  uint32_t getVerticesPerPoint();

private:
  struct RenderableInternals
  {
    inline bool bufferIsFull()
    {
      return current_vertex_count >= buffer_size;
    }

    inline bool noBufferOverflowOccurred()
    {
      return current_vertex_count == buffer_size;
    }

    PointCloudRenderablePtr rend;
    float * float_buffer = nullptr;
    uint32_t buffer_size = 0;
    Ogre::AxisAlignedBox aabb;
    uint32_t current_vertex_count = 0;
  };

  RVIZ_RENDERING_PUBLIC
  float * getVertices();

  RVIZ_RENDERING_PUBLIC
  Ogre::MaterialPtr getMaterialForRenderMode(RenderMode render_mode);

  RVIZ_RENDERING_PUBLIC
  bool changingGeometrySupportIsNecessary(const Ogre::MaterialPtr materialPtr);

  RVIZ_RENDERING_PUBLIC
  PointCloudRenderablePtr createRenderable(
    int num_points,
    Ogre::RenderOperation::OperationType
    operation_type);

  RVIZ_RENDERING_PUBLIC
  void regenerateAll();

  RVIZ_RENDERING_PUBLIC
  size_t removePointsFromRenderables(uint32_t number_of_points, uint32_t vertices_per_point);

  RVIZ_RENDERING_PUBLIC
  void resetBoundingBoxForCurrentPoints();

  RVIZ_RENDERING_PUBLIC
  RenderableInternals createNewRenderable(uint32_t number_of_points_to_be_added);

  RVIZ_RENDERING_PUBLIC
  Ogre::RenderOperation::OperationType getRenderOperationType() const;

  RVIZ_RENDERING_PUBLIC
  void finishRenderable(RenderableInternals internals, uint32_t vertex_count_of_renderable);

  RVIZ_RENDERING_PUBLIC
  uint32_t getColorForPoint(
    uint32_t current_point,
    std::vector<PointCloud::Point>::iterator point) const;

  RVIZ_RENDERING_PUBLIC
  RenderableInternals addPointToHardwareBuffer(
    RenderableInternals internals,
    std::vector<PointCloud::Point>::iterator point, uint32_t current_point);

  Ogre::AxisAlignedBox bounding_box_;       ///< The bounding box of this point cloud

  typedef std::vector<Point> V_Point;
  ///< The list of points we're displaying. Allocates to a high-water-mark.
  V_Point points_;
  uint32_t point_count_;                    ///< The number of points currently in #points_

  RenderMode render_mode_;
  Ogre::Vector4 point_extensions_;          ///< width, height, depth of particles
  Ogre::Vector3 common_direction_;          ///< See Ogre::BillboardSet::setCommonDirection
  Ogre::Vector3 common_up_vector_;          ///< See Ogre::BillboardSet::setCommonUpVector

  Ogre::MaterialPtr point_material_;
  Ogre::MaterialPtr square_material_;
  Ogre::MaterialPtr flat_square_material_;
  Ogre::MaterialPtr sphere_material_;
  Ogre::MaterialPtr tile_material_;
  Ogre::MaterialPtr box_material_;
  Ogre::MaterialPtr current_material_;
  float alpha_;

  bool color_by_index_;

  PointCloudRenderableQueue renderables_;

  bool current_mode_supports_geometry_shader_;
  Ogre::ColourValue pick_color_;

  static Ogre::String sm_Type;              ///< The "renderable type" used by Ogre
};

}  // namespace rviz_rendering

#endif  // RVIZ_RENDERING__OBJECTS__POINT_CLOUD_HPP_
