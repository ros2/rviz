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

// This file is adapted from jsk_rviz_plugins OverlayCameraDisplay (ROS 1).

#include "rviz_default_plugins/displays/overlay_camera/overlay_camera_display.hpp"

#include <OgreHardwarePixelBuffer.h>
#include <OgrePixelFormat.h>
#include <OgreRenderTexture.h>
#include <OgreRenderTarget.h>
#include <OgreResourceGroupManager.h>
#include <OgreTextureManager.h>
#include <OgreViewport.h>

#include <algorithm>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/int_property.hpp"
#include "rviz_common/uniform_string_stream.hpp"
#include "rviz_rendering/render_system.hpp"
#include "rviz_rendering/render_window.hpp"

namespace rviz_default_plugins
{
namespace displays
{

OverlayCameraDisplay::OverlayCameraDisplay()
: overlay_(nullptr),
  width_property_(new rviz_common::properties::IntProperty(
      "Width", 640, "Width of the overlay camera image.", this, SLOT(updateWidth()))),
  height_property_(new rviz_common::properties::IntProperty(
      "Height", 480, "Height of the overlay camera image.", this, SLOT(updateHeight()))),
  left_property_(new rviz_common::properties::IntProperty(
      "Left", 0, "Left position of the overlay camera image.", this, SLOT(updateLeft()))),
  top_property_(new rviz_common::properties::IntProperty(
      "Top", 0, "Top position of the overlay camera image.", this, SLOT(updateTop()))),
  texture_alpha_property_(new rviz_common::properties::FloatProperty(
      "Texture Alpha", 0.8f, "Opacity of the overlaid camera texture.", this,
      SLOT(updateTextureAlpha()))),
  width_(640),
  height_(480),
  left_(0),
  top_(0),
  texture_alpha_(0.8f),
  render_texture_width_(0),
  render_texture_height_(0),
  render_texture_name_(),
  render_texture_target_(nullptr),
  render_texture_viewport_(nullptr)
{
  width_property_->setMin(0);
  height_property_->setMin(0);
  texture_alpha_property_->setMin(0.0f);
  texture_alpha_property_->setMax(1.0f);
}

OverlayCameraDisplay::~OverlayCameraDisplay()
{
  destroyRenderTexture();
}

void OverlayCameraDisplay::onInitialize()
{
  CameraDisplay::onInitialize();

  rviz_rendering::RenderSystem::get()->prepareOverlays(scene_manager_);

  // Do not expose the inherited camera panel as a dock widget.
  setAssociatedWidget(nullptr);
  if (render_panel_) {
    render_panel_->hide();
  }
}

void OverlayCameraDisplay::onEnable()
{
  CameraDisplay::onEnable();
  ensureOverlay();
  if (overlay_) {
    overlay_->show();
  }
}

void OverlayCameraDisplay::onDisable()
{
  if (overlay_) {
    overlay_->hide();
  }
  destroyRenderTexture();
  CameraDisplay::onDisable();
}

void OverlayCameraDisplay::ensureOverlay()
{
  if (overlay_) {
    return;
  }
  static int count = 0;
  rviz_common::UniformStringStream stream;
  stream << "OverlayCameraDisplayObject" << count++;
  overlay_ = std::make_shared<OverlayObject>(stream.str());
}

void OverlayCameraDisplay::destroyRenderTexture()
{
  if (render_texture_target_) {
    render_texture_target_->removeListener(this);
    render_texture_target_->removeAllViewports();
    render_texture_target_ = nullptr;
    render_texture_viewport_ = nullptr;
  }

  if (!render_texture_name_.empty()) {
    auto & texture_manager = Ogre::TextureManager::getSingleton();
    if (texture_manager.resourceExists(render_texture_name_)) {
      texture_manager.remove(render_texture_name_);
    }
    render_texture_name_.clear();
  }

  render_texture_width_ = 0;
  render_texture_height_ = 0;
}

bool OverlayCameraDisplay::createOrUpdateRenderTexture()
{
  if (!render_panel_) {
    return false;
  }

  const auto desired_width = static_cast<uint32_t>(std::max(width_, 1));
  const auto desired_height = static_cast<uint32_t>(std::max(height_, 1));
  if (render_texture_target_ &&
    render_texture_viewport_ &&
    render_texture_width_ == desired_width &&
    render_texture_height_ == desired_height)
  {
    return true;
  }

  destroyRenderTexture();

  auto * render_window = render_panel_->getRenderWindow();
  if (!render_window) {
    return false;
  }

  Ogre::Camera * ogre_camera =
    rviz_rendering::RenderWindowOgreAdapter::getOgreCamera(render_window);
  if (!ogre_camera) {
    return false;
  }

  static int texture_count = 0;
  rviz_common::UniformStringStream stream;
  stream << "OverlayCameraDisplayRenderTexture" << texture_count++;
  render_texture_name_ = stream.str();

  Ogre::TexturePtr render_texture = Ogre::TextureManager::getSingleton().createManual(
    render_texture_name_,
    Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
    Ogre::TEX_TYPE_2D,
    desired_width,
    desired_height,
    0,
    Ogre::PF_A8R8G8B8,
    Ogre::TU_RENDERTARGET);

  render_texture_target_ = render_texture->getBuffer()->getRenderTarget();
  if (!render_texture_target_) {
    destroyRenderTexture();
    return false;
  }

  render_texture_viewport_ = render_texture_target_->addViewport(ogre_camera);
  if (!render_texture_viewport_) {
    destroyRenderTexture();
    return false;
  }

  uint32_t visibility_mask = std::numeric_limits<uint32_t>::max();
  auto * source_viewport = rviz_rendering::RenderWindowOgreAdapter::getOgreViewport(render_window);
  if (!source_viewport) {
    render_window->create();
    render_window->initialize();
    source_viewport = rviz_rendering::RenderWindowOgreAdapter::getOgreViewport(render_window);
  }
  if (source_viewport) {
    visibility_mask = source_viewport->getVisibilityMask();
  }
  render_texture_viewport_->setVisibilityMask(visibility_mask);

  // Ogre overlays must not be drawn into this target: our own overlay panel is
  // one of them, so leaving them enabled feeds the overlay texture back into
  // itself and every frame washes out towards flat grey.
  render_texture_viewport_->setOverlaysEnabled(false);

  render_texture_target_->setAutoUpdated(false);
  render_texture_target_->addListener(this);

  render_texture_width_ = desired_width;
  render_texture_height_ = desired_height;
  return true;
}

void OverlayCameraDisplay::redraw(Ogre::RenderTarget * render_target)
{
  if (!overlay_ || !render_target) {
    return;
  }

  const int texture_width = static_cast<int>(overlay_->getTextureWidth());
  const int texture_height = static_cast<int>(overlay_->getTextureHeight());
  if (texture_width <= 0 || texture_height <= 0) {
    return;
  }

  ScopedPixelBuffer buffer = overlay_->getBuffer();
  QImage image = buffer.getQImage(*overlay_);
  if (image.isNull()) {
    return;
  }

  const int target_width = static_cast<int>(render_target->getWidth());
  const int target_height = static_cast<int>(render_target->getHeight());
  if (target_width <= 0 || target_height <= 0) {
    return;
  }

  const int copy_width = std::min({texture_width, target_width, image.width()});
  const int copy_height = std::min({texture_height, target_height, image.height()});
  if (copy_width <= 0 || copy_height <= 0) {
    return;
  }

  std::vector<Ogre::uint32> pixel_data(
    static_cast<size_t>(target_width) * static_cast<size_t>(target_height));
  Ogre::PixelBox pixel_box(
    static_cast<size_t>(target_width), static_cast<size_t>(target_height), 1,
    Ogre::PF_A8R8G8B8, pixel_data.data());
  render_target->copyContentsToMemory(pixel_box);

  const size_t copy_row_bytes = static_cast<size_t>(copy_width) * sizeof(Ogre::uint32);
  for (int y = 0; y < copy_height; ++y) {
    const Ogre::uint32 * src =
      pixel_data.data() + static_cast<size_t>(y) * static_cast<size_t>(target_width);
    auto * dst = reinterpret_cast<Ogre::uint32 *>(image.scanLine(y));
    std::memcpy(dst, src, copy_row_bytes);
  }

  const int alpha = static_cast<int>(std::clamp(texture_alpha_, 0.0f, 1.0f) * 255.0f);
  const Ogre::uint32 alpha_bits = static_cast<Ogre::uint32>(alpha) << 24;
  const int alpha_width = std::min(texture_width, image.width());
  const int alpha_height = std::min(texture_height, image.height());
  for (int y = 0; y < alpha_height; ++y) {
    auto * row = reinterpret_cast<Ogre::uint32 *>(image.scanLine(y));
    for (int x = 0; x < alpha_width; ++x) {
      row[x] = (row[x] & 0x00FFFFFFu) | alpha_bits;
    }
  }
}

void OverlayCameraDisplay::update(std::chrono::nanoseconds wall_dt, std::chrono::nanoseconds ros_dt)
{
  CameraDisplay::update(wall_dt, ros_dt);

  if (!isEnabled() || !render_panel_) {
    return;
  }

  ensureOverlay();
  if (!overlay_) {
    return;
  }

  // reset() hides the overlay and runs whenever the topic is (re)applied, so the
  // show() in onEnable() is not enough to keep it visible.
  if (!overlay_->isVisible()) {
    overlay_->show();
  }

  if (!createOrUpdateRenderTexture() || !render_texture_target_) {
    return;
  }

  render_texture_target_->update();

  overlay_->updateTextureSize(
    render_texture_target_->getWidth(), render_texture_target_->getHeight());
  redraw(render_texture_target_);
  overlay_->setDimensions(width_, height_);
  overlay_->setPosition(left_, top_);
}

void OverlayCameraDisplay::reset()
{
  CameraDisplay::reset();
  if (overlay_) {
    overlay_->hide();
  }
}

void OverlayCameraDisplay::updateWidth()
{
  width_ = width_property_->getInt();
  context_->queueRender();
}

void OverlayCameraDisplay::updateHeight()
{
  height_ = height_property_->getInt();
  context_->queueRender();
}

void OverlayCameraDisplay::updateLeft()
{
  left_ = left_property_->getInt();
  context_->queueRender();
}

void OverlayCameraDisplay::updateTop()
{
  top_ = top_property_->getInt();
  context_->queueRender();
}

void OverlayCameraDisplay::updateTextureAlpha()
{
  texture_alpha_ = texture_alpha_property_->getFloat();
  context_->queueRender();
}

}  // namespace displays
}  // namespace rviz_default_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::OverlayCameraDisplay, rviz_common::Display)
