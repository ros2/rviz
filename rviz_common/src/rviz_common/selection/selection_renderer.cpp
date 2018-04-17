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

#include "rviz_common/selection/selection_renderer.hpp"

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

#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "rclcpp/publisher.hpp"

#include "rviz_rendering/custom_parameter_indices.hpp"
#include "rviz_rendering/render_window.hpp"

#include "rviz_common/logging.hpp"

#include "rviz_common/properties/property.hpp"
#include "rviz_common/properties/property_tree_model.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/view_manager.hpp"
#include "rviz_common/display_context.hpp"

namespace rviz_common
{
namespace selection
{
SelectionRenderer::SelectionRenderer()
: debug_mode_(false)
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
  rviz_common::DisplayContext * context,
  Ogre::Camera * camera,
  SelectionRectangle rectangle,
  RenderTexture texture,
  Ogre::PixelBox & dst_box)
{
  // TODO(Martin-Idel-SI): This is just a quick fix to be able to write tests.
  // Refactoring necessary
  Ogre::Viewport * viewport = rectangle.viewport_;
  int x1 = rectangle.x1_;
  int x2 = rectangle.x2_;
  int y1 = rectangle.y1_;
  int y2 = rectangle.y2_;

  Ogre::TexturePtr tex = texture.tex_;
  unsigned int texture_width = texture.texture_width_;
  unsigned int texture_height = texture.texture_height_;
  std::string material_scheme = texture.material_scheme_;

  context->lockRender();

  if (x1 > x2) {std::swap(x1, x2);}
  if (y1 > y2) {std::swap(y1, y2);}

  if (x1 < 0) {x1 = 0;}
  if (y1 < 0) {y1 = 0;}
  if (x1 > viewport->getActualWidth() - 2) {x1 = viewport->getActualWidth() - 2;}
  if (y1 > viewport->getActualHeight() - 2) {y1 = viewport->getActualHeight() - 2;}
  if (x2 < 0) {x2 = 0;}
  if (y2 < 0) {y2 = 0;}
  if (x2 > viewport->getActualWidth() - 2) {x2 = viewport->getActualWidth() - 2;}
  if (y2 > viewport->getActualHeight() - 2) {y2 = viewport->getActualHeight() - 2;}

  if (x2 == x1) {x2++;}
  if (y2 == y1) {y2++;}

  unsigned w = x2 - x1;
  unsigned h = y2 - y1;

  Ogre::HardwarePixelBufferSharedPtr pixel_buffer = tex->getBuffer();
  Ogre::RenderTexture * render_texture = pixel_buffer->getRenderTarget();

  Ogre::Matrix4 proj_matrix = viewport->getCamera()->getProjectionMatrix();
  Ogre::Matrix4 scale_matrix = Ogre::Matrix4::IDENTITY;
  Ogre::Matrix4 trans_matrix = Ogre::Matrix4::IDENTITY;

  float x1_rel = static_cast<float>(x1) / static_cast<float>(viewport->getActualWidth() - 1) - 0.5f;
  float y1_rel = static_cast<float>(y1) / static_cast<float>(viewport->getActualHeight() - 1) -
    0.5f;
  float x2_rel = static_cast<float>(x2) / static_cast<float>(viewport->getActualWidth() - 1) - 0.5f;
  float y2_rel = static_cast<float>(y2) / static_cast<float>(viewport->getActualHeight() - 1) -
    0.5f;

  scale_matrix[0][0] = 1.0 / (x2_rel - x1_rel);
  scale_matrix[1][1] = 1.0 / (y2_rel - y1_rel);

  trans_matrix[0][3] -= x1_rel + x2_rel;
  trans_matrix[1][3] += y1_rel + y2_rel;

  camera->setCustomProjectionMatrix(true, scale_matrix * trans_matrix * proj_matrix);

  camera->setPosition(viewport->getCamera()->getDerivedPosition());
  camera->setOrientation(viewport->getCamera()->getDerivedOrientation());

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

  unsigned render_w = w;
  unsigned render_h = h;

  if (w > h) {
    if (render_w > texture_width) {
      render_w = texture_width;
      render_h = round(
        static_cast<float>(h) * static_cast<float>(texture_width) / static_cast<float>(w));
    }
  } else {
    if (render_h > texture_height) {
      render_h = texture_height;
      render_w = round(
        static_cast<float>(w) * static_cast<float>(texture_height) / static_cast<float>(h));
    }
  }

  // safety clamping in case of rounding errors
  if (render_w > texture_width) {render_w = texture_width;}
  if (render_h > texture_height) {render_h = texture_height;}

  // set viewport to render to a subwindow of the texture
  Ogre::Viewport * render_viewport = render_texture->getViewport(0);
  render_viewport->setDimensions(
    0,
    0,
    static_cast<float>(render_w) / static_cast<float>(texture_width),
    static_cast<float>(render_h) / static_cast<float>(texture_height)
  );

  // make sure the same objects are visible as in the original viewport
  render_viewport->setVisibilityMask(viewport->getVisibilityMask());

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
  using rviz_rendering::RenderWindowOgreAdapter;
  Ogre::Viewport * main_view =
    RenderWindowOgreAdapter::getOgreViewport(context->getRenderPanel()->getRenderWindow());
  context->getSceneManager()->addRenderQueueListener(this);
  context->getSceneManager()->_renderScene(main_view->getCamera(), main_view, false);
  context->getSceneManager()->removeRenderQueueListener(this);

  Ogre::MaterialManager::getSingleton().removeListener(this);

  render_w = render_viewport->getActualWidth();
  render_h = render_viewport->getActualHeight();

  Ogre::PixelFormat format = pixel_buffer->getFormat();

  int size = static_cast<int>(Ogre::PixelUtil::getMemorySize(render_w, render_h, 1, format));
  uint8_t * data = new uint8_t[size];

  delete[] reinterpret_cast<uint8_t *>(dst_box.data);
  dst_box = Ogre::PixelBox(render_w, render_h, 1, format, data);

  pixel_buffer->blitToMemory(dst_box, dst_box);

  context->unlockRender();

  if (debug_mode_) {
    Ogre::Image image;
    image.loadDynamicImage(static_cast<uint8_t *>(dst_box.data),
      dst_box.getWidth(), dst_box.getHeight(), 1, format);
    image.save("select" + material_scheme + ".png");
  }

  return true;
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
      return NULL;
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
      return NULL;
    }
  }
}

void SelectionRenderer::setDebugMode(bool debug)
{
  debug_mode_ = debug;
}

}  // namespace selection
}  // namespace rviz_common
