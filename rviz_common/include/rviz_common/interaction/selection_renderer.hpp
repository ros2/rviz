/*
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
 *     * Neither the name of the copyright holder nor the names of its
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

#ifndef RVIZ_COMMON__INTERACTION__SELECTION_RENDERER_HPP_
#define RVIZ_COMMON__INTERACTION__SELECTION_RENDERER_HPP_

#include <map>
#include <memory>
#include <string>

#include <OgreMaterialManager.h>
#include <OgreRenderQueueListener.h>

#include "rviz_rendering/render_window.hpp"
#include "rviz_common/interaction/handler_manager_iface.hpp"
#include "rviz_common/visibility_control.hpp"

namespace rviz_common
{
class DisplayContext;

namespace interaction
{

struct SelectionRectangle
{
  SelectionRectangle(int x1, int y1, int x2, int y2)
  : x1(x1), x2(x2), y1(y1), y2(y2)
  {}

  ~SelectionRectangle() = default;

  int x1;
  int x2;
  int y1;
  int y2;
};

struct Dimensions
{
  Dimensions()
  : width(0), height(0) {}
  Dimensions(float width, float height)
  : width(width), height(height) {}

  float width;
  float height;
};

struct RenderTexture
{
  RenderTexture(
    Ogre::TexturePtr tex,
    Dimensions dimensions,
    std::string material_scheme)
  : tex(tex),
    dimensions(dimensions),
    material_scheme(material_scheme)
  {}

  ~RenderTexture() = default;

  Ogre::TexturePtr tex;
  Dimensions dimensions;
  std::string material_scheme;
};

class SelectionRenderer
  : public Ogre::MaterialManager::Listener, public Ogre::RenderQueueListener
{
public:
  RVIZ_COMMON_PUBLIC explicit SelectionRenderer(rviz_common::DisplayContext * context);
  ~SelectionRenderer() override = default;

  RVIZ_COMMON_PUBLIC
  virtual void initialize(Ogre::Camera * camera);

  RVIZ_COMMON_PUBLIC
  virtual void render(
    rviz_rendering::RenderWindow * window,
    SelectionRectangle rectangle,
    RenderTexture texture,
    HandlerRange handlers,
    Ogre::PixelBox & dst_box);

  /// Implementation for Ogre::RenderQueueListener.
  RVIZ_COMMON_PUBLIC
  void renderQueueStarted(
    uint8_t queueGroupId,
    const std::string & invocation,
    bool & skipThisInvocation) override;

  /// Implementation for Ogre::MaterialManager::Listener
  /// If a material does not support the picking scheme, paint it black.
  RVIZ_COMMON_PUBLIC
  Ogre::Technique * handleSchemeNotFound(
    unsigned short scheme_index,  // NOLINT: Ogre decides the use of unsigned short
    const Ogre::String & scheme_name,
    Ogre::Material * original_material,
    unsigned short lod_index,  // NOLINT: Ogre decides the use of unsigned short
    const Ogre::Renderable * rend) override;

private:
  void sanitizeRectangle(Ogre::Viewport * viewport, SelectionRectangle & rectangle);

  void configureCamera(Ogre::Viewport * viewport, const SelectionRectangle & rectangle) const;
  float getRelativeCoordinate(float coord, int dimension) const;

  template<typename T>
  T clamp(T value, T min, T max) const;

  Ogre::RenderTexture * setupRenderTexture(
    Ogre::HardwarePixelBufferSharedPtr pixel_buffer,
    RenderTexture texture) const;

  Ogre::Viewport * setupRenderViewport(
    Ogre::RenderTexture * render_texture,
    Ogre::Viewport * viewport,
    const SelectionRectangle & rectangle,
    Dimensions texture_dimensions);

  Dimensions getRenderDimensions(
    const SelectionRectangle & rectangle,
    const Dimensions & texture_dim) const;

  void renderToTexture(Ogre::RenderTexture * render_texture, Ogre::Viewport * window_viewport);

  void blitToMemory(
    Ogre::HardwarePixelBufferSharedPtr & pixel_buffer,
    const Ogre::Viewport * render_viewport,
    Ogre::PixelBox & dst_box) const;

  rviz_common::DisplayContext * context_;

  Ogre::Camera * camera_;
  Ogre::SceneNode * camera_node_;

  Ogre::MaterialPtr fallback_pick_material_;
  Ogre::Technique * fallback_pick_technique_;
  Ogre::Technique * fallback_black_technique_;
  Ogre::Technique * fallback_depth_technique_;
  Ogre::Technique * fallback_pick_cull_technique_;
  Ogre::Technique * fallback_black_cull_technique_;
  Ogre::Technique * fallback_depth_cull_technique_;
};

}  // namespace interaction
}  // namespace rviz_common

#endif  // RVIZ_COMMON__INTERACTION__SELECTION_RENDERER_HPP_
