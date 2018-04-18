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

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

#include <OgreMaterialManager.h>
#include <OgreRenderQueueListener.h>

#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "rviz_common/visibility_control.hpp"

namespace rviz_common
{
class DisplayContext;

namespace interaction
{

struct SelectionRectangle
{
  SelectionRectangle(Ogre::Viewport * viewport, int x1, int y1, int x2, int y2)
  : viewport_(viewport), x1_(x1), x2_(x2), y1_(y1), y2_(y2)
  {}

  ~SelectionRectangle() = default;

  Ogre::Viewport * viewport_;
  int x1_;
  int x2_;
  int y1_;
  int y2_;
};

struct Dimensions
{
  Dimensions(): width(0), height(0) {}
  Dimensions(float width, float height): width(width), height(height) {}

  float width;
  float height;
};

struct RenderTexture
{
  RenderTexture(
    Ogre::TexturePtr tex,
    Dimensions dimensions,
    std::string material_scheme)
  : tex_(tex),
    dimensions_(dimensions),
    material_scheme_(material_scheme)
  {}

  ~RenderTexture() = default;

  Ogre::TexturePtr tex_;
  Dimensions dimensions_;
  std::string material_scheme_;
};

class SelectionRenderer
  : public Ogre::MaterialManager::Listener, public Ogre::RenderQueueListener
{
public:
  RVIZ_COMMON_PUBLIC explicit SelectionRenderer(rviz_common::DisplayContext * context);
  ~SelectionRenderer() override = default;

  RVIZ_COMMON_PUBLIC
  virtual void initialize();

  RVIZ_COMMON_PUBLIC
  virtual bool render(
    Ogre::Camera * camera,
    SelectionRectangle rectangle,
    RenderTexture texture,
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

  RVIZ_COMMON_PUBLIC
  void setDebugMode(bool debug);

private:
  // void publishDebugImage(const Ogre::PixelBox & pixel_box, const std::string & label);

  void sanitizeRectangle(SelectionRectangle & rectangle) const;

  void configureCamera(Ogre::Camera * camera, const SelectionRectangle & rectangle) const;
  float getRelativeCoordinate(float coord, int dimension) const;

  template<typename T> T clamp(T value, T min, T max) const;

  Ogre::RenderTexture * setupRenderTexture(
    Ogre::HardwarePixelBufferSharedPtr pixel_buffer,
    Ogre::Camera * camera,
    RenderTexture texture) const;

  Ogre::Viewport * setupViewport(
    Ogre::RenderTexture * render_texture,
    const SelectionRectangle & rectangle,
    Dimensions texture_dimensions);

  Dimensions getRenderDimensions(
    const SelectionRectangle & rectangle,
    const Dimensions & texture_dim) const;

  void renderToTexture(Ogre::RenderTexture * render_texture);

  void blitToMemory(
    Ogre::HardwarePixelBufferSharedPtr & pixel_buffer,
    const Ogre::Viewport * render_viewport,
    Ogre::PixelBox & dst_box) const;

  rviz_common::DisplayContext * context_;

  Ogre::MaterialPtr fallback_pick_material_;
  Ogre::Technique * fallback_pick_technique_;
  Ogre::Technique * fallback_black_technique_;
  Ogre::Technique * fallback_depth_technique_;
  Ogre::Technique * fallback_pick_cull_technique_;
  Ogre::Technique * fallback_black_cull_technique_;
  Ogre::Technique * fallback_depth_cull_technique_;

  bool debug_mode_;
};

}  // namespace interaction
}  // namespace rviz_common

#endif  // RVIZ_COMMON__INTERACTION__SELECTION_RENDERER_HPP_
