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

#include "rviz_common/interaction/selection_renderer.hpp"

#include <string>
#include <utility>

#include <OgreCamera.h>
#include <OgreHardwarePixelBuffer.h>
#include <OgreMaterialManager.h>
#include <OgreRectangle2D.h>
#include <OgreRenderTexture.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTechnique.h>
#include <OgreTextureManager.h>
#include <OgreViewport.h>

#include <QTimer>  // NOLINT: cpplint is unable to handle the include order here

#include "rviz_rendering/render_window.hpp"

#include "rviz_common/display_context.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/view_manager.hpp"

namespace rviz_common
{
namespace interaction
{
SelectionRenderer::SelectionRenderer(rviz_common::DisplayContext * context)
: context_(context),
  camera_(nullptr),
  camera_node_(nullptr)
{}

void SelectionRenderer::initialize(Ogre::Camera * camera)
{
  camera_ = camera;
  camera_node_ = camera_->getParentSceneNode();

  fallback_pick_material_ = Ogre::MaterialManager::getSingleton().getByName(
    "rviz/DefaultPickAndDepth");
  if (fallback_pick_material_) {
    fallback_pick_material_->load();

    fallback_pick_cull_technique_ = fallback_pick_material_->getTechnique("PickCull");
    fallback_black_cull_technique_ = fallback_pick_material_->getTechnique("BlackCull");
    fallback_depth_cull_technique_ = fallback_pick_material_->getTechnique("DepthCull");

    fallback_pick_technique_ = fallback_pick_material_->getTechnique("Pick");
    fallback_black_technique_ = fallback_pick_material_->getTechnique("Black");
    fallback_depth_technique_ = fallback_pick_material_->getTechnique("Depth");
  } else {
    RVIZ_COMMON_LOG_ERROR("failed to load material 'rviz/DefaultPickAndDepth'");
  }
}

void SelectionRenderer::render(
  rviz_rendering::RenderWindow * window,
  SelectionRectangle rectangle,
  RenderTexture texture,
  HandlerRange handlers,
  Ogre::PixelBox & dst_box)
{
  context_->lockRender();
  for (const auto & handler : handlers) {
    handler.lock()->preRenderPass(0);
  }

  auto window_viewport = rviz_rendering::RenderWindowOgreAdapter::getOgreViewport(window);

  sanitizeRectangle(window_viewport, rectangle);

  configureCamera(window_viewport, rectangle);

  Ogre::HardwarePixelBufferSharedPtr pixel_buffer = texture.tex->getBuffer();
  auto render_texture = setupRenderTexture(pixel_buffer, texture);
  auto render_viewport =
    setupRenderViewport(render_texture, window_viewport, rectangle, texture.dimensions);

  renderToTexture(render_texture, window_viewport);

  blitToMemory(pixel_buffer, render_viewport, dst_box);

  context_->unlockRender();
  for (const auto & handler : handlers) {
    handler.lock()->postRenderPass(0);
  }
}


void SelectionRenderer::sanitizeRectangle(Ogre::Viewport * viewport, SelectionRectangle & rectangle)
{
  int & x1 = rectangle.x1;
  int & x2 = rectangle.x2;
  int & y1 = rectangle.y1;
  int & y2 = rectangle.y2;

  if (x1 > x2) {std::swap(x1, x2);}
  if (y1 > y2) {std::swap(y1, y2);}

  x1 = clamp(x1, 0, viewport->getActualWidth() - 2);
  x2 = clamp(x2, 0, viewport->getActualWidth() - 2);
  y1 = clamp(y1, 0, viewport->getActualHeight() - 2);
  y2 = clamp(y2, 0, viewport->getActualHeight() - 2);

  if (x2 == x1) {x2++;}
  if (y2 == y1) {y2++;}
}

template<typename T>
T SelectionRenderer::clamp(T value, T min, T max) const
{
  if (value < min) {value = min;} else if (value > max) {value = max;}
  return value;
}

void SelectionRenderer::configureCamera(
  Ogre::Viewport * viewport, const SelectionRectangle & rectangle) const
{
  Ogre::Matrix4 proj_matrix = viewport->getCamera()->getProjectionMatrix();
  Ogre::Matrix4 scale_matrix = Ogre::Matrix4::IDENTITY;
  Ogre::Matrix4 trans_matrix = Ogre::Matrix4::IDENTITY;

  float x1_rel = getRelativeCoordinate(rectangle.x1, viewport->getActualWidth());
  float y1_rel = getRelativeCoordinate(rectangle.y1, viewport->getActualHeight());
  float x2_rel = getRelativeCoordinate(rectangle.x2, viewport->getActualWidth());
  float y2_rel = getRelativeCoordinate(rectangle.y2, viewport->getActualHeight());

  scale_matrix[0][0] = 1.f / (x2_rel - x1_rel);
  scale_matrix[1][1] = 1.f / (y2_rel - y1_rel);

  trans_matrix[0][3] -= x1_rel + x2_rel;
  trans_matrix[1][3] += y1_rel + y2_rel;

  camera_->setCustomProjectionMatrix(true, scale_matrix * trans_matrix * proj_matrix);

  camera_node_->setPosition(viewport->getCamera()->getDerivedPosition());
  camera_node_->setOrientation(viewport->getCamera()->getDerivedOrientation());
}

float SelectionRenderer::getRelativeCoordinate(float coordinate, int dimension) const
{
  return coordinate / static_cast<float>(dimension - 1) - 0.5f;
}

Ogre::RenderTexture * SelectionRenderer::setupRenderTexture(
  Ogre::HardwarePixelBufferSharedPtr pixel_buffer,
  RenderTexture texture) const
{
  Ogre::RenderTexture * render_texture = pixel_buffer->getRenderTarget();

  // create a viewport if there is none
  if (render_texture->getNumViewports() == 0) {
    render_texture->removeAllViewports();
    render_texture->addViewport(camera_);
    Ogre::Viewport * render_viewport = render_texture->getViewport(0);
    render_viewport->setClearEveryFrame(true);
    render_viewport->setBackgroundColour(Ogre::ColourValue::Black);
    render_viewport->setOverlaysEnabled(false);
    render_viewport->setMaterialScheme(texture.material_scheme);
  }
  return render_texture;
}

Ogre::Viewport * SelectionRenderer::setupRenderViewport(
  Ogre::RenderTexture * render_texture,
  Ogre::Viewport * viewport,
  const SelectionRectangle & rectangle,
  Dimensions texture_dimensions)
{
  Dimensions render_dimensions = getRenderDimensions(rectangle, texture_dimensions);

  // set viewport to render to a subwindow of the texture
  Ogre::Viewport * render_viewport = render_texture->getViewport(0);
  render_viewport->setDimensions(
    0,
    0,
    render_dimensions.width / texture_dimensions.width,
    render_dimensions.height / texture_dimensions.height
  );

  // make sure the same objects are visible as in the original viewport
  render_viewport->setVisibilityMask(viewport->getVisibilityMask());
  return render_viewport;
}

Dimensions SelectionRenderer::getRenderDimensions(
  const SelectionRectangle & rectangle,
  const Dimensions & texture_dim) const
{
  Dimensions selection(
    rectangle.x2 - rectangle.x1,
    rectangle.y2 - rectangle.y1
  );

  Dimensions render_dimensions = selection;

  if (selection.width > selection.height) {
    if (render_dimensions.width > texture_dim.width) {
      render_dimensions.width = texture_dim.width;
      render_dimensions.height = round(selection.height * texture_dim.width / selection.width);
    }
  } else {
    if (render_dimensions.height > texture_dim.height) {
      render_dimensions.height = texture_dim.height;
      render_dimensions.width = round(selection.width * texture_dim.height / selection.height);
    }
  }

  // safety clamping in case of rounding errors
  render_dimensions.width = clamp(render_dimensions.width, 0.f, texture_dim.width);
  render_dimensions.height = clamp(render_dimensions.height, 0.f, texture_dim.height);
  return render_dimensions;
}

void SelectionRenderer::renderToTexture(
  Ogre::RenderTexture * render_texture, Ogre::Viewport * window_viewport)
{
  (void)window_viewport;
  // update & force ogre to render the scene
  Ogre::MaterialManager::getSingleton().addListener(this);
  render_texture->update();
  Ogre::MaterialManager::getSingleton().removeListener(this);
}

void SelectionRenderer::blitToMemory(
  Ogre::HardwarePixelBufferSharedPtr & pixel_buffer,
  const Ogre::Viewport * render_viewport,
  Ogre::PixelBox & dst_box) const
{
  auto viewport_w = static_cast<unsigned>(render_viewport->getActualWidth());
  auto viewport_h = static_cast<unsigned>(render_viewport->getActualHeight());

  auto format = pixel_buffer->getFormat();
  auto size = Ogre::PixelUtil::getMemorySize(viewport_w, viewport_h, 1, format);
  auto data = new uint8_t[size];

  delete[] static_cast<uint8_t *>(dst_box.data);
  dst_box = Ogre::PixelBox(viewport_w, viewport_h, 1, format, data);

  pixel_buffer->blitToMemory(dst_box, dst_box);
}

void SelectionRenderer::renderQueueStarted(
  uint8_t queueGroupId,
  const std::string & invocation,
  bool & skipThisInvocation)
{
  Q_UNUSED(queueGroupId);
  Q_UNUSED(invocation);
  // This render queue listener function tells the scene manager to
  // skip every render step, so nothing actually gets drawn.
  skipThisInvocation = true;
}

Ogre::Technique * SelectionRenderer::handleSchemeNotFound(
  unsigned short scheme_index,  // NOLINT: Ogre decides the use of unsigned short
  const Ogre::String & scheme_name,
  Ogre::Material * original_material,
  unsigned short lod_index,  // NOLINT: Ogre decides the use of unsigned short
  const Ogre::Renderable * rend)
{
  Q_UNUSED(scheme_index);
  Q_UNUSED(lod_index);
  // Find the original culling mode
  Ogre::CullingMode culling_mode = Ogre::CULL_CLOCKWISE;
  Ogre::Technique * orig_tech = original_material->getTechnique(0);
  if (orig_tech && orig_tech->getNumPasses() > 0) {
    culling_mode = orig_tech->getPass(0)->getCullingMode();
  }

  // find out if the renderable has the picking param set
  bool has_pick_param = false;
  if (rend != nullptr) {
    has_pick_param = rend->getUserObjectBindings().getUserAny("pick_handle").has_value();
  }

  // NOTE: it is important to avoid changing the culling mode of the
  // fallback techniques here, because that change then propagates to
  // other uses of these fallback techniques in unpredictable ways.
  // If you want to change the technique programmatically (like with
  // Pass::setCullingMode()), make sure you do it on a cloned material
  // which doesn't get shared with other objects.

  // Use the technique with the right name and culling mode.
  if (culling_mode == Ogre::CULL_CLOCKWISE) {
    if (scheme_name == "Pick") {
      return has_pick_param ? fallback_pick_cull_technique_ : fallback_black_cull_technique_;
    } else if (scheme_name == "Depth") {
      return fallback_depth_cull_technique_;
    }
    if (scheme_name == "Pick1") {
      return fallback_black_cull_technique_;
    } else {
      return nullptr;
    }
  } else {  // Must be CULL_NONE because we never use CULL_ANTICLOCKWISE
    if (scheme_name == "Pick") {
      return has_pick_param ? fallback_pick_technique_ : fallback_black_technique_;
    } else if (scheme_name == "Depth") {
      return fallback_depth_technique_;
    }
    if (scheme_name == "Pick1") {
      return fallback_black_technique_;
    } else {
      return nullptr;
    }
  }
}

}  // namespace interaction
}  // namespace rviz_common
