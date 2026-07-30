// Copyright (c) 2022, Team Spatzenhirn
// Copyright (c) 2014, JSK Lab
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This file is adapted from JSK Lab's jsk_rviz_plugins overlay_utils and
// rviz_2d_overlay_plugins overlay_utils (BSD-3-Clause).

#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__OVERLAY__OVERLAY_UTILS_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__OVERLAY__OVERLAY_UTILS_HPP_

#include <OgreHardwarePixelBuffer.h>
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>
#include <OgreTexture.h>
#include <OgreTextureManager.h>
#include <Overlay/OgreOverlay.h>
#include <Overlay/OgreOverlayContainer.h>
#include <Overlay/OgreOverlayElement.h>
#include <Overlay/OgreOverlayManager.h>
#include <Overlay/OgrePanelOverlayElement.h>

#include <QColor>
#include <QImage>

#include <cstdint>
#include <memory>
#include <string>

#include "rviz_default_plugins/visibility_control.hpp"

namespace rviz_default_plugins
{
namespace displays
{

class OverlayObject;

class RVIZ_DEFAULT_PLUGINS_OVERLAY_PUBLIC ScopedPixelBuffer
{
public:
  explicit ScopedPixelBuffer(Ogre::HardwarePixelBufferSharedPtr pixel_buffer);
  ~ScopedPixelBuffer();

  Ogre::HardwarePixelBufferSharedPtr getPixelBuffer();
  QImage getQImage(unsigned int width, unsigned int height);
  QImage getQImage(OverlayObject & overlay);
  QImage getQImage(unsigned int width, unsigned int height, QColor & bg_color);
  QImage getQImage(OverlayObject & overlay, QColor & bg_color);

private:
  Ogre::HardwarePixelBufferSharedPtr pixel_buffer_;
};

// Helper class for placing a 2D panel overlay on top of the rviz render window.
class RVIZ_DEFAULT_PLUGINS_OVERLAY_PUBLIC OverlayObject
{
public:
  using SharedPtr = std::shared_ptr<OverlayObject>;

  explicit OverlayObject(const std::string & name);
  ~OverlayObject();

  std::string getName() const;
  /// Ogre draw order of this overlay; higher means drawn on top.
  uint16_t getZOrder() const;
  void hide();
  void show();
  bool isTextureReady() const;
  void updateTextureSize(unsigned int width, unsigned int height);
  ScopedPixelBuffer getBuffer();
  void setPosition(double left, double top);
  void setDimensions(double width, double height);
  bool isVisible() const;
  unsigned int getTextureWidth() const;
  unsigned int getTextureHeight() const;

private:
  const std::string name_;
  Ogre::Overlay * overlay_;
  Ogre::PanelOverlayElement * panel_;
  Ogre::MaterialPtr panel_material_;
  Ogre::TexturePtr texture_;
  uint16_t z_order_;
};

}  // namespace displays
}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__OVERLAY__OVERLAY_UTILS_HPP_
