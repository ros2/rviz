/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * Copyright (c) 2017, Open Source Robotics Foundation, Inc.
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

#ifndef RVIZ_COMMON__INTERACTION__SELECTION_MANAGER_HPP_
#define RVIZ_COMMON__INTERACTION__SELECTION_MANAGER_HPP_

#include "selection_manager_iface.hpp"

#include <array>
#include <memory>
#include <mutex>
#include <utility>

#include <OgreMaterialManager.h>
#include <OgreRenderQueueListener.h>

#include <QObject>  // NOLINT: cpplint is unable to handle the include order here

#include "rviz_common/interaction/forwards.hpp"
#include "rviz_common/interaction/handler_manager_iface.hpp"
#include "rviz_common/interaction/handler_manager_listener.hpp"
#include "rviz_common/interaction/selection_handler.hpp"
#include "rviz_common/visibility_control.hpp"

namespace Ogre
{
class Rectangle2D;
}

namespace rviz_common
{

namespace properties
{
class PropertyTreeModel;
}

class VisualizationManager;

namespace interaction
{
class SelectionRenderer;
struct SelectionRectangle;
struct RenderTexture;

class RVIZ_COMMON_PUBLIC SelectionManager
  : public SelectionManagerIface, public HandlerManagerListener
{
  Q_OBJECT

public:
  SelectionManager(DisplayContext * manager, std::shared_ptr<SelectionRenderer> renderer);

  explicit SelectionManager(DisplayContext * manager);

  ~SelectionManager() override;

  void initialize() override;

  /// Control the highlight box being displayed while selecting.
  void highlight(rviz_rendering::RenderWindow * window, int x1, int y1, int x2, int y2) override;

  void removeHighlight() override;

  /// Select all objects in bounding box.
  void select(
    rviz_rendering::RenderWindow * window,
    int x1,
    int y1,
    int x2,
    int y2,
    SelectType type) override;

  /// Get all objects in a bounding box.
  void pick(
    rviz_rendering::RenderWindow * window,
    int x1,
    int y1,
    int x2,
    int y2,
    M_Picked & results) override;

  void update() override;

  const M_Picked & getSelection() const override;

  static Ogre::ColourValue handleToColor(CollObjectHandle handle);

  // static CollObjectHandle colourToHandle( const Ogre::ColourValue & color );

  // TODO(Martin-Idel-SI): maybe move those methods to SelectionRenderer
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

  /// Tell the view controller to look at the selection.
  void focusOnSelection() override;

  /// Change the size of the off-screen selection buffer texture.
  void setTextureSize(unsigned size) override;

  rviz_common::properties::PropertyTreeModel * getPropertyModel() override;

  void onHandlerRemoved(CollObjectHandle handle) override;

private Q_SLOTS:
  /// Call updateProperties() on all SelectionHandlers in the current selection.
  void updateProperties();

private:
  /// Set the list of currently selected objects.
  void setSelection(const M_Picked & objs);

  void addSelection(const M_Picked & objs);

  void removeSelection(const M_Picked & objs);

  void selectionAdded(const M_Picked & added);

  void selectionRemoved(const M_Picked & removed);

  std::pair<Picked, bool> addSelectedObject(const Picked & obj);

  void removeSelectedObject(const Picked & obj);

  void setHighlightRect(Ogre::Viewport * viewport, int x1, int y1, int x2, int y2);

  /// Render to a texture for one of the picking passes and unpack the resulting pixels into
  // pixel_buffer_.
  void renderAndUnpack(
    rviz_rendering::RenderWindow * window,
    const SelectionRectangle & selection_rectangle,
    uint32_t pass);

  /// Internal render function to render to a texture and read the pixels back out.
  void render(
    rviz_rendering::RenderWindow * window,
    const SelectionRectangle & selection_rectangle,
    const RenderTexture & render_texture,
    Ogre::PixelBox & dst_box);

  /// Unpacks a pixelbox into pixel_buffer_
  void unpackColors(const Ogre::PixelBox & box);

  void setUpSlots();


  DisplayContext * context_;
  std::shared_ptr<HandlerManagerIface> handler_manager_;

  std::recursive_mutex selection_mutex_;

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
  std::array<Ogre::TexturePtr, kNumRenderTextures_> render_textures_;
  std::array<Ogre::PixelBox, kNumRenderTextures_> pixel_boxes_;

  Ogre::Rectangle2D * highlight_rectangle_;
  Ogre::SceneNode * highlight_node_;
  Ogre::Camera * camera_;

  V_CollObject pixel_buffer_;

  uint32_t texture_size_;

  rviz_common::properties::PropertyTreeModel * property_model_;

  std::shared_ptr<rviz_common::interaction::SelectionRenderer> renderer_;
};

}  // namespace interaction
}  // namespace rviz_common

#endif  // RVIZ_COMMON__INTERACTION__SELECTION_MANAGER_HPP_
