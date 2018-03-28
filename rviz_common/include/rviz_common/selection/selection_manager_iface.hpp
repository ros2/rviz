/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * Copyright (c) 2017, Open Source Robotics Foundation, Inc.
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

#ifndef RVIZ_COMMON__SELECTION__SELECTION_MANAGER_IFACE_HPP_
#define RVIZ_COMMON__SELECTION__SELECTION_MANAGER_IFACE_HPP_

#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <QObject>  // NOLINT: cpplint is unable to handle the include order here

#include "./forwards.hpp"
#include "./selection_handler.hpp"
#include "rviz_common/visibility_control.hpp"

namespace Ogre
{
class Rectangle2D;
}  // namespace Ogre

namespace rviz_rendering
{
class RenderWindow;
}  // namespace rviz_rendering

namespace rviz_common
{
namespace properties
{
class PropertyTreeModel;
}

class DisplayContext;

namespace selection
{

class RVIZ_COMMON_PUBLIC SelectionManagerIface
  : public QObject
{
  Q_OBJECT

public:
  enum SelectType
  {
    Add,
    Remove,
    Replace
  };

  virtual void
  initialize() = 0;

  /// Enables or disables publishing of picking and depth rendering images.
  virtual void
  setDebugMode(bool debug) = 0;

  virtual void
  clearHandlers() = 0;

  virtual void
  addObject(CollObjectHandle obj, SelectionHandler * handler) = 0;

  virtual void
  removeObject(CollObjectHandle obj) = 0;

  /// Control the highlight box being displayed while selecting.
  virtual void
  highlight(rviz_rendering::RenderWindow * window, int x1, int y1, int x2, int y2) = 0;

  virtual void
  removeHighlight() = 0;

  /// Select all objects in bounding box.
  virtual void
  select(
    rviz_rendering::RenderWindow * window, int x1, int y1, int x2, int y2, SelectType type) = 0;

  /**
   * \return handles of all objects in the given bounding box
   * \param single_render_pass only perform one rendering pass
   *   (point cloud selecting won't work)
   */
  virtual void
  pick(
    rviz_rendering::RenderWindow * window,
    int x1,
    int y1,
    int x2,
    int y2,
    M_Picked & results,
    bool single_render_pass = false) = 0;

  virtual void update() = 0;

  /// Set the list of currently selected objects.
  virtual void setSelection(const M_Picked & objs) = 0;

  virtual void addSelection(const M_Picked & objs) = 0;

  virtual void removeSelection(const M_Picked & objs) = 0;

  virtual const M_Picked & getSelection() const = 0;

  virtual SelectionHandler * getHandler(CollObjectHandle obj) = 0;

  /// Create a new unique handle.
  virtual CollObjectHandle createHandle() = 0;

  /// Tell all handlers that interactive mode is active/inactive.
  virtual void enableInteraction(bool enable) = 0;

  virtual bool getInteractionEnabled() const = 0;

  /// Tell the view controller to look at the selection.
  virtual void focusOnSelection() = 0;

  /// Change the size of the off-screen selection buffer texture.
  virtual void setTextureSize(unsigned size) = 0;

  /// Return true if the point at x, y in the viewport is showing an object, false otherwise.
  /**
   * If it is showing an object, result will be changed to contain the 3D point
   * corresponding to it.
   */
  virtual bool get3DPoint(
    Ogre::Viewport * viewport,
    int x,
    int y,
    Ogre::Vector3 & result_point) = 0;

  /// Gets the 3D points in a box around a point in a view port.
  /**
   * \param[in] viewport Rendering area clicked on.
   * \param[in] x x coordinate of upper-left corner of box.
   * \param[in] y y coordinate of upper-left corner of box.
   * \param[in] width The width of the rendered box in pixels.
   * \param[in] height The height of the rendered box in pixels.
   * \param[in] skip_missing Whether to skip non-existing points or insert
   *   NaNs for them
   * \param[out] result_points The vector of output points.
   *
   * \returns True if any valid point is rendered in the box. NaN points count,
   *   so if skip_missing is false, this will always return true if
   *   width and height are > 0.
   */
  virtual bool get3DPatch(
    Ogre::Viewport * viewport,
    int x,
    int y,
    unsigned width,
    unsigned height,
    bool skip_missing,
    std::vector<Ogre::Vector3> & result_points) = 0;


  /// Renders a depth image in a box around a point in a view port.
  /**
   * \param[in] viewport Rendering area clicked on.
   * \param[in] x x coordinate of upper-left corner of box.
   * \param[in] y y coordinate of upper-left corner of box.
   * \param[in] width The width of the rendered box in pixels.
   * \param[in] height The height of the rendered box in pixels.
   * \param[out] depth_vector The vector of depth values.
   *
   * \returns True if rendering operation to render depth data to the depth
   *   texture buffer succeeds.
   *   Failure likely indicates a pretty serious problem.
   */
  virtual bool getPatchDepthImage(
    Ogre::Viewport * viewport,
    int x,
    int y,
    unsigned width,
    unsigned height,
    std::vector<float> & depth_vector) = 0;


  virtual rviz_common::properties::PropertyTreeModel * getPropertyModel() = 0;
};

}  // namespace selection
}  // namespace rviz_common

#endif  // RVIZ_COMMON__SELECTION__SELECTION_MANAGER_IFACE_HPP_
