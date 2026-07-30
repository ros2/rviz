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

#include "rviz_default_plugins/displays/overlay/overlay_utils.hpp"

#include <atomic>
#include <cstring>
#include <string>

#include "rviz_common/logging.hpp"

namespace rviz_default_plugins
{
namespace displays
{

namespace
{

// Ogre draws overlays in ascending z-order and every overlay defaults to the
// same value, which leaves the stacking of two overlays undefined. Handing out
// an increasing z-order makes it deterministic (later overlay on top) and lets
// OverlayPickerTool pick the one the user actually sees. Ogre asserts z-order
// stays below 650.
constexpr uint16_t kFirstZOrder = 100U;
constexpr uint16_t kLastZOrder = 640U;

uint16_t nextZOrder()
{
  static std::atomic<unsigned int> counter{0U};
  const unsigned int index = counter++ % (kLastZOrder - kFirstZOrder + 1U);
  return static_cast<uint16_t>(kFirstZOrder + index);
}

}  // namespace

ScopedPixelBuffer::ScopedPixelBuffer(Ogre::HardwarePixelBufferSharedPtr pixel_buffer)
: pixel_buffer_(pixel_buffer)
{
  if (pixel_buffer_) {
    pixel_buffer_->lock(Ogre::HardwareBuffer::HBL_NORMAL);
  }
}

ScopedPixelBuffer::~ScopedPixelBuffer()
{
  if (pixel_buffer_) {
    pixel_buffer_->unlock();
  }
}

Ogre::HardwarePixelBufferSharedPtr ScopedPixelBuffer::getPixelBuffer()
{
  return pixel_buffer_;
}

QImage ScopedPixelBuffer::getQImage(unsigned int width, unsigned int height)
{
  if (!pixel_buffer_) {
    return QImage();
  }

  const Ogre::PixelBox & pixel_box = pixel_buffer_->getCurrentLock();
  auto * destination = static_cast<Ogre::uint8 *>(pixel_box.data);
  // Ogre only guarantees rowPitch >= width; a backend is free to pad rows to a
  // hardware alignment. Both the clear and the QImage wrapper have to use the
  // real stride, otherwise the image comes out sheared on such a backend.
  const size_t bytes_per_line = static_cast<size_t>(pixel_box.rowPitch) * 4U;
  std::memset(destination, 0, bytes_per_line * height);
  return QImage(
    destination, static_cast<int>(width), static_cast<int>(height),
    static_cast<qsizetype>(bytes_per_line), QImage::Format_ARGB32);
}

QImage ScopedPixelBuffer::getQImage(unsigned int width, unsigned int height, QColor & bg_color)
{
  QImage image = getQImage(width, height);
  if (!image.isNull()) {
    image.fill(bg_color.rgba());
  }
  return image;
}

QImage ScopedPixelBuffer::getQImage(OverlayObject & overlay)
{
  return getQImage(overlay.getTextureWidth(), overlay.getTextureHeight());
}

QImage ScopedPixelBuffer::getQImage(OverlayObject & overlay, QColor & bg_color)
{
  return getQImage(overlay.getTextureWidth(), overlay.getTextureHeight(), bg_color);
}

OverlayObject::OverlayObject(const std::string & name)
: name_(name),
  overlay_(nullptr),
  panel_(nullptr),
  z_order_(nextZOrder())
{
  const std::string material_name = name_ + "Material";
  auto * overlay_manager = Ogre::OverlayManager::getSingletonPtr();
  overlay_ = overlay_manager->create(name_);
  overlay_->setZOrder(z_order_);
  panel_ = static_cast<Ogre::PanelOverlayElement *>(
    overlay_manager->createOverlayElement("Panel", name_ + "Panel"));
  panel_->setMetricsMode(Ogre::GMM_PIXELS);

  panel_material_ = Ogre::MaterialManager::getSingleton().create(
    material_name,
    Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  panel_->setMaterialName(panel_material_->getName());
  overlay_->add2D(panel_);
}

OverlayObject::~OverlayObject()
{
  if (texture_ != nullptr) {
    Ogre::TextureManager::getSingleton().remove(texture_);
    texture_.reset();
  }

  auto * overlay_manager = Ogre::OverlayManager::getSingletonPtr();
  if (overlay_manager) {
    if (panel_ != nullptr) {
      overlay_manager->destroyOverlayElement(panel_);
      panel_ = nullptr;
    }
    if (overlay_ != nullptr) {
      overlay_manager->destroy(overlay_);
      overlay_ = nullptr;
    }
  }

  if (panel_material_) {
    panel_material_->unload();
    Ogre::MaterialManager::getSingleton().remove(panel_material_->getName());
  }
}

std::string OverlayObject::getName() const
{
  return name_;
}

uint16_t OverlayObject::getZOrder() const
{
  return z_order_;
}

void OverlayObject::hide()
{
  if (overlay_ != nullptr && overlay_->isVisible()) {
    overlay_->hide();
  }
}

void OverlayObject::show()
{
  if (overlay_ != nullptr && !overlay_->isVisible()) {
    overlay_->show();
  }
}

bool OverlayObject::isTextureReady() const
{
  return texture_ != nullptr;
}

void OverlayObject::updateTextureSize(unsigned int width, unsigned int height)
{
  const std::string texture_name = name_ + "Texture";

  if (width == 0) {
    RVIZ_COMMON_LOG_WARNING_STREAM("[OverlayObject] width=0 is specified as texture size");
    width = 1;
  }

  if (height == 0) {
    RVIZ_COMMON_LOG_WARNING_STREAM("[OverlayObject] height=0 is specified as texture size");
    height = 1;
  }

  if (!isTextureReady() || (width != texture_->getWidth()) || (height != texture_->getHeight())) {
    if (isTextureReady()) {
      Ogre::TextureManager::getSingleton().remove(texture_);
      panel_material_->getTechnique(0)->getPass(0)->removeAllTextureUnitStates();
    }

    texture_ = Ogre::TextureManager::getSingleton().createManual(
      texture_name,
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
      Ogre::TEX_TYPE_2D,
      width,
      height,
      0,
      Ogre::PF_A8R8G8B8,
      Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);

    Ogre::Pass * pass = panel_material_->getTechnique(0)->getPass(0);
    pass->createTextureUnitState(texture_name);
    pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    pass->setDepthWriteEnabled(false);
    pass->setDepthCheckEnabled(false);
  }
}

ScopedPixelBuffer OverlayObject::getBuffer()
{
  if (isTextureReady()) {
    return ScopedPixelBuffer(texture_->getBuffer());
  }
  return ScopedPixelBuffer(Ogre::HardwarePixelBufferSharedPtr());
}

void OverlayObject::setPosition(double left, double top)
{
  if (panel_ != nullptr) {
    panel_->setPosition(left, top);
  }
}

void OverlayObject::setDimensions(double width, double height)
{
  if (panel_ != nullptr) {
    panel_->setDimensions(width, height);
  }
}

bool OverlayObject::isVisible() const
{
  return overlay_ != nullptr && overlay_->isVisible();
}

unsigned int OverlayObject::getTextureWidth() const
{
  return isTextureReady() ? texture_->getWidth() : 0U;
}

unsigned int OverlayObject::getTextureHeight() const
{
  return isTextureReady() ? texture_->getHeight() : 0U;
}

}  // namespace displays
}  // namespace rviz_default_plugins
