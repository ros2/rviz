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

#ifndef RVIZ_COMMON__SELECTION__SELECTION_MANAGER_HPP_
#define RVIZ_COMMON__SELECTION__SELECTION_MANAGER_HPP_

#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

#include <OgreMaterialManager.h>
#include <OgreRenderQueueListener.h>

#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include <QObject>  // NOLINT: cpplint is unable to handle the include order here

#include "./forwards.hpp"
#include "./selection_handler.hpp"

namespace rclcpp
{

class PublisherBase;

}

namespace Ogre
{

class Rectangle2D;

}  // namespace Ogre

namespace rviz_common
{

namespace properties
{

class PropertyTreeModel;

}

class VisualizationManager;

namespace selection
{

class SelectionManager
  : public QObject,
  public Ogre::MaterialManager::Listener,
  public Ogre::RenderQueueListener
{
  Q_OBJECT

public:
  enum SelectType
  {
    Add,
    Remove,
    Replace
  };

  explicit SelectionManager(VisualizationManager * manager);

  virtual ~SelectionManager();

  void
  initialize();

  /// Enables or disables publishing of picking and depth rendering images.
  void
  setDebugMode(bool debug);

  void
  clearHandlers();

  void
  addObject(CollObjectHandle obj, SelectionHandler * handler);

  void
  removeObject(CollObjectHandle obj);

  /// Control the highlight box being displayed while selecting.
  void
  highlight(Ogre::Viewport * viewport, int x1, int y1, int x2, int y2);

  void
  removeHighlight();

  /// Select all objects in bounding box.
  void
  select(Ogre::Viewport * viewport, int x1, int y1, int x2, int y2, SelectType type);

  /**
   * \return handles of all objects in the given bounding box
   * \param single_render_pass only perform one rendering pass
   *   (point cloud selecting won't work)
   */
  void
  pick(
    Ogre::Viewport * viewport,
    int x1,
    int y1,
    int x2,
    int y2,
    M_Picked & results,
    bool single_render_pass = false);

  void update();

  /// Set the list of currently selected objects.
  void setSelection(const M_Picked & objs);

  void addSelection(const M_Picked & objs);

  void removeSelection(const M_Picked & objs);

  const M_Picked & getSelection() const;

  SelectionHandler * getHandler(CollObjectHandle obj);

  static Ogre::ColourValue handleToColor(CollObjectHandle handle);

  // static CollObjectHandle colourToHandle( const Ogre::ColourValue & color );

  static void setPickColor(const Ogre::ColourValue & color, Ogre::SceneNode * node);

  static void setPickColor(const Ogre::ColourValue & color, Ogre::MovableObject * object);

  static void setPickHandle(CollObjectHandle handle, Ogre::SceneNode * node);

  static void setPickHandle(CollObjectHandle handle, Ogre::MovableObject * object);

  static void setPickData(
    CollObjectHandle handle,
    const Ogre::ColourValue & color,
    Ogre::SceneNode * node);

  static void setPickData(
    CollObjectHandle handle,
    const Ogre::ColourValue & color,
    Ogre::MovableObject * object);

  /// If a material does not support the picking scheme, paint it black.
  Ogre::Technique * handleSchemeNotFound(
    unsigned short scheme_index,  // NOLINT: Ogre decides the use of unsigned short
    const Ogre::String & scheme_name,
    Ogre::Material * original_material,
    unsigned short lod_index,  // NOLINT: Ogre decides the use of unsigned short
    const Ogre::Renderable * rend) override;

  /// Create a new unique handle.
  CollObjectHandle createHandle();

  /// Tell all handlers that interactive mode is active/inactive.
  void enableInteraction(bool enable);

  bool getInteractionEnabled() const;

  /// Tell the view controller to look at the selection.
  void focusOnSelection();

  /// Change the size of the off-screen selection buffer texture.
  void setTextureSize(unsigned size);

  /// Return true if the point at x, y in the viewport is showing an object, false otherwise.
  /**
   * If it is showing an object, result will be changed to contain the 3D point
   * corresponding to it.
   */
  bool get3DPoint(
    Ogre::Viewport * viewport,
    const int x,
    const int y,
    Ogre::Vector3 & result_point);

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
  bool get3DPatch(
    Ogre::Viewport * viewport,
    const int x,
    const int y,
    const unsigned width,
    const unsigned height,
    const bool skip_missing,
    std::vector<Ogre::Vector3> & result_points);


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
  bool getPatchDepthImage(
    Ogre::Viewport * viewport,
    const int x,
    const int y,
    const unsigned width,
    const unsigned height,
    std::vector<float> & depth_vector);

  /// Implementation for Ogre::RenderQueueListener.
  void renderQueueStarted(
    uint8_t queueGroupId,
    const std::string & invocation,
    bool & skipThisInvocation) override;

  rviz_common::properties::PropertyTreeModel * getPropertyModel();

private Q_SLOTS:
  /// Call updateProperties() on all SelectionHandlers in the current selection.
  void updateProperties();

private:
  void selectionAdded(const M_Picked & added);

  void selectionRemoved(const M_Picked & removed);

  std::pair<Picked, bool> addSelectedObject(const Picked & obj);

  void removeSelectedObject(const Picked & obj);

  void setHighlightRect(Ogre::Viewport * viewport, int x1, int y1, int x2, int y2);

  /// Render to a texture for one of the picking passes and unpack the resulting pixels.
  void renderAndUnpack(
    Ogre::Viewport * viewport,
    uint32_t pass,
    int x1,
    int y1,
    int x2,
    int y2,
    V_CollObject & pixels);

  /// Internal render function to render to a texture and read the pixels back out.
  bool render(
    Ogre::Viewport * viewport,
    Ogre::TexturePtr tex,
    int x1,
    int y1,
    int x2,
    int y2,
    Ogre::PixelBox & dst_box,
    std::string material_scheme,
    unsigned texture_width,
    unsigned textured_height);

  void unpackColors(const Ogre::PixelBox & box, V_CollObject & pixels);

  void setDepthTextureSize(unsigned width, unsigned height);

  void publishDebugImage(const Ogre::PixelBox & pixel_box, const std::string & label);

  VisualizationManager * vis_manager_;

  std::recursive_mutex global_mutex_;

  typedef std::unordered_map<CollObjectHandle, SelectionHandler *>
    M_CollisionObjectToSelectionHandler;
  M_CollisionObjectToSelectionHandler objects_;

  bool highlight_enabled_;

  struct Highlight
  {
    int x1;
    int y1;
    int x2;
    int y2;
    Ogre::Viewport * viewport;
  };
  Highlight highlight_;

  M_Picked selection_;

  // If you want to change this number to something > 3 you must provide more
  // width for extra handles in the Picked structure (currently a u64)
  static constexpr uint32_t kNumRenderTextures_ = 2;
  Ogre::TexturePtr render_textures_[kNumRenderTextures_];
  Ogre::PixelBox pixel_boxes_[kNumRenderTextures_];

  // Graphics card -based depth finding of clicked points.
  Ogre::TexturePtr depth_render_texture_;
  uint32_t depth_texture_width_, depth_texture_height_;
  Ogre::PixelBox depth_pixel_box_;

  uint32_t uid_counter_;

  Ogre::Rectangle2D * highlight_rectangle_;
  Ogre::SceneNode * highlight_node_;
  Ogre::Camera * camera_;

  V_CollObject pixel_buffer_;

  bool interaction_enabled_;

  bool debug_mode_;

  Ogre::MaterialPtr fallback_pick_material_;
  Ogre::Technique * fallback_pick_technique_;
  Ogre::Technique * fallback_black_technique_;
  Ogre::Technique * fallback_depth_technique_;
  Ogre::Technique * fallback_pick_cull_technique_;
  Ogre::Technique * fallback_black_cull_technique_;
  Ogre::Technique * fallback_depth_cull_technique_;

  uint32_t texture_size_;

  rviz_common::properties::PropertyTreeModel * property_model_;

  typedef std::map<std::string, std::shared_ptr<rclcpp::PublisherBase>> PublisherMap;
  PublisherMap debug_publishers_;
};

}  // namespace selection
}  // namespace rviz_common

#endif  // RVIZ_COMMON__SELECTION__SELECTION_MANAGER_HPP_
