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

#include <algorithm>
#include <cassert>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

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

#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include <QTimer>  // NOLINT: cpplint is unable to handle the include order here

#include "rclcpp/publisher.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "rviz_rendering/custom_parameter_indices.hpp"
#include "rviz_rendering/render_window.hpp"

#include "rviz_common/display_context.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/properties/property.hpp"
#include "rviz_common/properties/property_tree_model.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/view_manager.hpp"

namespace rviz_common
{
namespace interaction
{
SelectionRenderer::SelectionRenderer(rviz_common::DisplayContext * context)
: context_(context), 
  debug_mode_(false)
{}

void SelectionRenderer::initialize()
{
  fallback_pick_material_ = Ogre::MaterialManager::getSingleton().getByName(
    "rviz/DefaultPickAndDepth");
  // TODO(wjwwood): see why this fails to load
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

// TODO(Martin-Idel-SI): It seems that rendering cannot return false (I deleted the only "false"
// path because it was irrelevant). In that case, I'd much rather not have an output parameter.
bool SelectionRenderer::render(
  Ogre::Camera * camera,
  SelectionRectangle rectangle,
  RenderTexture texture,
  Ogre::PixelBox & dst_box)
{
  context_->lockRender();

  sanitizeRectangle(rectangle);

  configureCamera(camera, rectangle);

  Ogre::TexturePtr tex = texture.tex_;
  std::string material_scheme = texture.material_scheme_;
  Ogre::HardwarePixelBufferSharedPtr pixel_buffer = tex->getBuffer();
  Ogre::RenderTexture * render_texture = pixel_buffer->getRenderTarget();

  // create a viewport if there is none
  if (render_texture->getNumViewports() == 0) {
    render_texture->removeAllViewports();
    render_texture->addViewport(camera);
    Ogre::Viewport * render_viewport = render_texture->getViewport(0);
    render_viewport->setClearEveryFrame(true);
    render_viewport->setBackgroundColour(Ogre::ColourValue::Black);
    render_viewport->setOverlaysEnabled(false);
    render_viewport->setMaterialScheme(material_scheme);
  }

  Dimensions texture_dimensions(texture.texture_width_, texture.texture_height_);
  Ogre::Viewport * render_viewport = setupViewport(render_texture, rectangle, texture_dimensions);

  renderToTexture(render_texture);

  auto viewport_w = static_cast<unsigned>(render_viewport->getActualWidth());
  auto viewport_h = static_cast<unsigned>(render_viewport->getActualHeight());

  Ogre::PixelFormat format = pixel_buffer->getFormat();

  auto size = Ogre::PixelUtil::getMemorySize(viewport_w, viewport_h, 1, format);
  auto data = new uint8_t[size];

  delete[] reinterpret_cast<uint8_t *>(dst_box.data);
  dst_box = Ogre::PixelBox(viewport_w, viewport_h, 1, format, data);

  pixel_buffer->blitToMemory(dst_box, dst_box);

  context_->unlockRender();

  if (debug_mode_) {
    Ogre::Image image;
    image.loadDynamicImage(static_cast<uint8_t *>(dst_box.data),
      dst_box.getWidth(), dst_box.getHeight(), 1, format);
    image.save("select" + material_scheme + ".png");
  }

  return true;
}

void SelectionRenderer::configureCamera(
  Ogre::Camera * camera,
  const SelectionRectangle & rectangle) const
{
  Ogre::Viewport * viewport = rectangle.viewport_;
  Ogre::Matrix4 proj_matrix = viewport->getCamera()->getProjectionMatrix();
  Ogre::Matrix4 scale_matrix = Ogre::Matrix4::IDENTITY;
  Ogre::Matrix4 trans_matrix = Ogre::Matrix4::IDENTITY;

  float x1_rel = calculateRelativeCoordinate(rectangle.x1_, viewport->getActualWidth());
  float y1_rel = calculateRelativeCoordinate(rectangle.y1_, viewport->getActualHeight());
  float x2_rel = calculateRelativeCoordinate(rectangle.x2_, viewport->getActualWidth());
  float y2_rel = calculateRelativeCoordinate(rectangle.y2_, viewport->getActualHeight());

  scale_matrix[0][0] = 1.f / (x2_rel - x1_rel);
  scale_matrix[1][1] = 1.f / (y2_rel - y1_rel);

  trans_matrix[0][3] -= x1_rel + x2_rel;
  trans_matrix[1][3] += y1_rel + y2_rel;

  camera->setCustomProjectionMatrix(true, scale_matrix * trans_matrix * proj_matrix);

  camera->setPosition(viewport->getCamera()->getDerivedPosition());
  camera->setOrientation(viewport->getCamera()->getDerivedOrientation());
}

Ogre::Viewport * SelectionRenderer::setupViewport(
  Ogre::RenderTexture * render_texture,
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
  render_viewport->setVisibilityMask(rectangle.viewport_->getVisibilityMask());
  return render_viewport;
}

Dimensions SelectionRenderer::getRenderDimensions(
  const SelectionRectangle & rectangle,
  const Dimensions & texture_dim) const
{
  Dimensions screen_dim (
    rectangle.x2_ - rectangle.x1_,
    rectangle.y2_ - rectangle.y1_
  );

  Dimensions render_dim = screen_dim;

  if (screen_dim.width > screen_dim.height) {
    if (render_dim.width > texture_dim.width) {
      render_dim.width = texture_dim.width;
      render_dim.height = round(screen_dim.height * texture_dim.width / screen_dim.width);
    }
  } else {
    if (render_dim.height > texture_dim.height) {
      render_dim.height = texture_dim.height;
      render_dim.width = round(screen_dim.width * texture_dim.height / screen_dim.height);
    }
  }

  // safety clamping in case of rounding errors
  render_dim.width = clamp(render_dim.width, 0.f, texture_dim.width);
  render_dim.height = clamp(render_dim.height, 0.f, texture_dim.height);
  return render_dim;
}

void SelectionRenderer::sanitizeRectangle(SelectionRectangle & rectangle) const
{
  int & x1 = rectangle.x1_;
  int & x2 = rectangle.x2_;
  int & y1 = rectangle.y1_;
  int & y2 = rectangle.y2_;

  if (x1 > x2) { std::swap(x1, x2);}
  if (y1 > y2) { std::swap(y1, y2);}

  x1 = clamp(x1, 0, rectangle.viewport_->getActualWidth() - 2);
  x2 = clamp(x2, 0, rectangle.viewport_->getActualWidth() - 2);
  y1 = clamp(y1, 0, rectangle.viewport_->getActualHeight() - 2);
  y2 = clamp(y2, 0, rectangle.viewport_->getActualHeight() - 2);

  if (x2 == x1) {x2++;}
  if (y2 == y1) {y2++;}
}

int SelectionRenderer::clamp(int value, int min, int max) const
{
  if (value < min) { value = min;}
  else if (value > max) { value = max;}
  return value;
}

float SelectionRenderer::clamp(float value, float min, float max) const
{
  if (value < min) { value = min;}
  else if (value > max) { value = max;}
  return value;
}

void SelectionRenderer::renderToTexture(Ogre::RenderTexture * render_texture)
{
  // update & force ogre to render the scene
  Ogre::MaterialManager::getSingleton().addListener(this);

  render_texture->update();

  // For some reason we need to pretend to render the main window in
  // order to get the picking render to show up in the pixelbox below.
  // If we don't do this, it will show up there the *next* time we
  // pick something, but not this time.  This object as a
  // render queue listener tells the scene manager to skip every
  // render step, so nothing actually gets drawn.
  //
  // TODO(unknown): find out what part of _renderScene() actually makes this work.
  Ogre::Viewport * main_view = rviz_rendering::RenderWindowOgreAdapter::getOgreViewport(
    context_->getRenderPanel()->getRenderWindow());
  context_->getSceneManager()->addRenderQueueListener(this);
  context_->getSceneManager()->_renderScene(main_view->getCamera(), main_view, false);
  context_->getSceneManager()->removeRenderQueueListener(this);

  Ogre::MaterialManager::getSingleton().removeListener(this);
}

float SelectionRenderer::calculateRelativeCoordinate(float coordinate, int width_height) const
{
  return coordinate / static_cast<float>(width_height - 1) - 0.5f;
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
  bool has_pick_param = rend->getUserObjectBindings().getUserAny("pick_handle").has_value();

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

void SelectionRenderer::setDebugMode(bool debug)
{
  debug_mode_ = debug;
}

}  // namespace interaction
}  // namespace rviz_common
