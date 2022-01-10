/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

#include "ogre_render_window_impl.hpp"

#include <cstdlib>
#include <functional>

#include "OgreEntity.h"
#include "OgreCamera.h"
#include "OgreGpuProgramManager.h"
#include "OgreMaterialManager.h"
#include "OgreRenderWindow.h"
#include "OgreRoot.h"
#include "OgreSceneNode.h"
#include "OgreStringConverter.h"
#include "OgreTechnique.h"
#include "OgreTextureManager.h"
#include "OgreViewport.h"

#include "rviz_rendering/orthographic.hpp"
#include "rviz_rendering/render_system.hpp"
#include "rviz_rendering/objects/grid.hpp"
#include "rviz_rendering/logging.hpp"

namespace rviz_rendering
{

RenderWindowImpl::RenderWindowImpl(QWindow * parent)
: parent_(parent),
  render_system_(nullptr),
  ogre_render_window_(nullptr),
  ogre_frame_listener_(nullptr),
  ogre_scene_manager_(nullptr),
  ogre_camera_(nullptr),
  animating_(false),
  ogre_viewport_(nullptr),
  ortho_scale_(1.0f),
  pending_listeners_()
  // auto_render_(true),
  // overlays_enabled_(true),    // matches the default of Ogre::Viewport.
  // stereo_enabled_(false),
  // rendering_stereo_(false),
  // left_camera_(0),
  // right_camera_(0),
  // right_viewport_(0)
{
  RenderSystem::get();  // side-effect is that the render system is setup
  // this->initialize();
  // render_window_->setVisible(true);
  // render_window_->setAutoUpdated(true);

// #if OGRE_STEREO_ENABLE
//   viewport_->setDrawBuffer(Ogre::CBT_BACK);
// #endif
//   enableStereo(true);

  // setCameraAspectRatio();
}

RenderWindowImpl::~RenderWindowImpl()
{
  if (ogre_render_window_) {
    Ogre::Root::getSingletonPtr()->detachRenderTarget(ogre_render_window_);
    Ogre::Root::getSingletonPtr()->destroyRenderTarget(ogre_render_window_);
    // enableStereo(false);  // free stereo resources
  }
}

void
RenderWindowImpl::screenShot(Ogre::String imageName)
{
  ogre_render_window_->writeContentsToFile(imageName);
}

void
RenderWindowImpl::render()
{
  // How we tied in the render function for OGre3D with QWindow's render function.
  // This is what gets call repeatedly.
  // Note that we don't call this function directly; rather we use the renderNow()
  // function to call this method as we don't want to render the Ogre3D scene
  // unless everything is set up first. That is what renderNow() does.
  //
  // Theoretically you can have one function that does this check but from my
  // experience it seems better to keep things separate and keep the render
  // function as simple as possible.

  // At this point, Ogre::WindowEventUtilities::messagePump() was called previously.
  // This function would process native platform messages for each render window. However, this
  // should be done by Qt for us. If the behavior is different, consider reimplementing the
  // method using Qt onboard features.
  if (ogre_render_window_->isClosed()) {
    RVIZ_RENDERING_LOG_ERROR("in RenderSystemImpl::render() - ogre window is closed");
    return;
  }
  if (!render_system_->getOgreRoot()->renderOneFrame()) {
    RVIZ_RENDERING_LOG_WARNING("in RenderSystemImpl::render() - renderOneFrame() returned false");
  }
}

void
RenderWindowImpl::renderLater()
{
  parent_->requestUpdate();

  // Alternative impl?:

  // // This function forces QWindow to keep rendering.
  // // Omitting this causes the renderNow() function to only get called when the
  // // window is resized, moved, etc. as opposed to all of the time; which is
  // // generally what we need.
  // if (!m_update_pending)
  // {
  //   m_update_pending = true;
  //   QApplication::postEvent(this, new QEvent(QEvent::UpdateRequest));
  // }
}


// The renderNow() function calls the initialize() function when needed and
// if the QWindow is already initialized and prepped calls the render() method.
void
RenderWindowImpl::renderNow()
{
  if (!parent_->isExposed()) {
    return;
  }

  if (!render_system_ || !ogre_render_window_) {
    this->initialize();
    if (setup_scene_callback_) {
      setup_scene_callback_(ogre_scene_manager_->getRootSceneNode()->createChildSceneNode());
      setup_scene_callback_ = 0;
    }
  }

  this->render();

  if (animating_) {
    this->renderLater();
  }
}

void
RenderWindowImpl::setupSceneAfterInit(setupSceneCallback setup_scene_callback)
{
  if (render_system_) {
    setup_scene_callback(ogre_scene_manager_->getRootSceneNode()->createChildSceneNode());
  } else {
    setup_scene_callback_ = setup_scene_callback;
  }
}

void RenderWindowImpl::addListener(Ogre::RenderTargetListener * listener)
{
  if (ogre_render_window_) {
    ogre_render_window_->addListener(listener);
  } else {
    pending_listeners_.emplace_back(listener);
  }
}
void RenderWindowImpl::removeListener(Ogre::RenderTargetListener * listener)
{
  if (ogre_render_window_) {
    ogre_render_window_->removeListener(listener);
  } else {
    pending_listeners_.erase(
      std::find(pending_listeners_.begin(), pending_listeners_.end(), listener));
  }
}

void
RenderWindowImpl::initialize()
{
  render_system_ = RenderSystem::get();
  double pixel_ratio = parent_->devicePixelRatio();
  ogre_render_window_ = render_system_->makeRenderWindow(
    parent_->winId(), parent_->width(), parent_->height(), pixel_ratio);

  Ogre::Root * ogre_root = render_system_->getOgreRoot();
  if (!ogre_root) {
    auto msg = "Ogre::Root * is unexpectedly nullptr";
    RVIZ_RENDERING_LOG_ERROR(msg);
    throw std::runtime_error(msg);
  }

  if (!ogre_scene_manager_) {
    ogre_scene_manager_ = ogre_root->createSceneManager();

    ogre_directional_light_ = ogre_scene_manager_->createLight("MainDirectional");
    ogre_directional_light_->setType(Ogre::Light::LT_DIRECTIONAL);
    ogre_directional_light_->setDirection(Ogre::Vector3(-1, 0, -1));
    ogre_directional_light_->setDiffuseColour(Ogre::ColourValue(1.0f, 1.0f, 1.0f));

    ogre_camera_ = ogre_scene_manager_->createCamera("MainCamera");
    ogre_camera_->setNearClipDistance(0.1f);
    ogre_camera_->setFarClipDistance(200.0f);

    auto camera_node_ = ogre_scene_manager_->getRootSceneNode()->createChildSceneNode();
    ogre_camera_->setPosition(Ogre::Vector3(0.0f, 10.0f, 10.0f));
    ogre_camera_->lookAt(Ogre::Vector3(0.0f, 0.0f, 0.0f));
    camera_node_->attachObject(ogre_camera_);
  }

  if (ogre_camera_) {
    ogre_viewport_ = ogre_render_window_->addViewport(ogre_camera_);
    auto bg_color = Ogre::ColourValue(0.937254902f, 0.921568627f, 0.905882353f);  // Qt background
    ogre_viewport_->setBackgroundColour(bg_color);

    ogre_camera_->setAspectRatio(
      Ogre::Real(ogre_render_window_->getWidth()) / Ogre::Real(ogre_render_window_->getHeight()));
    ogre_camera_->setAutoAspectRatio(true);

    Ogre::TextureManager::getSingleton().setDefaultNumMipmaps(5);
    Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();
  }

  if (!pending_listeners_.empty()) {
    for (auto listener : pending_listeners_) {
      ogre_render_window_->addListener(listener);
    }
  }
  if (!pending_visibility_masks_.empty()) {
    for (auto mask : pending_visibility_masks_) {
      ogre_viewport_->setVisibilityMask(mask);
    }
  }
}

void
RenderWindowImpl::resize(size_t width, size_t height)
{
  if (ogre_render_window_) {
    this->setCameraAspectRatio();
    ogre_render_window_->resize(
      static_cast<unsigned int>(width),  // NOLINT
      static_cast<unsigned int>(height)  // NOLINT
    );
    ogre_render_window_->windowMovedOrResized();
  }
  this->renderLater();
}

#if 0
bool RenderWindowImpl::enableStereo(bool enable)
{
  bool was_enabled = stereo_enabled_;
  stereo_enabled_ = enable;
  setupStereo();
  return was_enabled;
}
#endif

#if 0
void RenderWindowImpl::setupStereo()
{
  bool render_stereo = stereo_enabled_ && RenderSystem::get()->isStereoSupported();

  if (render_stereo == rendering_stereo_) {
    return;
  }

  rendering_stereo_ = render_stereo;

  if (rendering_stereo_) {
    right_viewport_ = render_window_->addViewport(nullptr, 1);
#if OGRE_STEREO_ENABLE
    right_viewport_->setDrawBuffer(Ogre::CBT_BACK_RIGHT);
    viewport_->setDrawBuffer(Ogre::CBT_BACK_LEFT);
#endif

    setOverlaysEnabled(overlays_enabled_);
    setBackgroundColor(background_color_);
    if (camera_) {
      setCamera(camera_);
    }

    // addListener causes preViewportUpdate() to be called when rendering.
    render_window_->addListener(this);
  } else {
    render_window_->removeListener(this);
    render_window_->removeViewport(1);
    right_viewport_ = nullptr;

#if OGRE_STEREO_ENABLE
    viewport_->setDrawBuffer(Ogre::CBT_BACK);
#endif

    if (left_camera_) {
      left_camera_->getSceneManager()->destroyCamera(left_camera_);
    }
    left_camera_ = nullptr;
    if (right_camera_) {
      right_camera_->getSceneManager()->destroyCamera(right_camera_);
    }
    right_camera_ = nullptr;
  }
}
#endif

#if 0
// this is called just before rendering either viewport when stereo is enabled.
void RenderWindowImpl::preViewportUpdate(
  const Ogre::RenderTargetViewportEvent & evt)
{
  Ogre::Viewport * viewport = evt.source;

  const Ogre::Vector2 & offset = camera_->getFrustumOffset();
  const Ogre::Vector3 pos = camera_->getPosition();
  const Ogre::Vector3 right = camera_->getRight();
  const Ogre::Vector3 up = camera_->getUp();

  if (viewport == right_viewport_) {
    if (camera_->getProjectionType() != Ogre::PT_PERSPECTIVE || !right_camera_) {
      viewport->setCamera(camera_);
      return;
    }

    Ogre::Vector3 newpos = pos +
      right * offset.x +
      up * offset.y;

    right_camera_->synchroniseBaseSettingsWith(camera_);
    right_camera_->setFrustumOffset(-offset);
    right_camera_->setPosition(newpos);
    viewport->setCamera(right_camera_);
  } else if (viewport == viewport_) {
    if (camera_->getProjectionType() != Ogre::PT_PERSPECTIVE || !left_camera_) {
      viewport->setCamera(camera_);
      return;
    }

    Ogre::Vector3 newpos = pos -
      right * offset.x -
      up * offset.y;

    left_camera_->synchroniseBaseSettingsWith(camera_);
    left_camera_->setFrustumOffset(offset);
    left_camera_->setPosition(newpos);
    viewport->setCamera(left_camera_);
  } else {
    ROS_WARN("Begin rendering to unknown viewport.");
  }
}
#endif

#if 0
void RenderWindowImpl::postViewportUpdate(
  const Ogre::RenderTargetViewportEvent & evt)
{
  Ogre::Viewport * viewport = evt.source;

  if (viewport == right_viewport_) {
    // nothing to do here
  } else if (viewport == viewport_) {
    viewport->setCamera(camera_);
  } else {
    ROS_WARN("End rendering to unknown viewport.");
  }

  if (!right_camera_->isCustomProjectionMatrixEnabled()) {
    right_camera_->synchroniseBaseSettingsWith(camera_);
    right_camera_->setFrustumOffset(-camera_->getFrustumOffset());
  }
  right_viewport_->setCamera(right_camera_);
}
#endif

Ogre::Viewport * RenderWindowImpl::getViewport() const
{
  return ogre_viewport_;
}

void RenderWindowImpl::setCamera(Ogre::Camera * ogre_camera)
{
  if (ogre_camera) {
    ogre_camera_ = ogre_camera;
    if (ogre_viewport_) {
      ogre_viewport_->setCamera(ogre_camera);
      this->setCameraAspectRatio();
    }

    // if (ogre_camera_ && rendering_stereo_ && !left_ogre_camera_) {
    //   left_ogre_camera_ =
    //     ogre_camera_->getSceneManager()->createCamera(ogre_camera_->getName() + "-left");
    // }
    // if (ogre_camera_ && rendering_stereo_ && !right_ogre_camera_) {
    //   right_ogre_camera_ =
    //     ogre_camera_->getSceneManager()->createCamera(ogre_camera_->getName() + "-right");
    // }

    // TODO(wjwwood): not sure where this was going before, need to figure that out
    //                and see if it is still needed, could have been QWidget before
    // this->update();
  }
}

Ogre::Camera * RenderWindowImpl::getCamera() const
{
  return ogre_camera_;
}

Ogre::Light * RenderWindowImpl::getDirectionalLight() const
{
  return ogre_directional_light_;
}

Ogre::SceneManager * RenderWindowImpl::getSceneManager() const
{
  return ogre_scene_manager_;
}

void RenderWindowImpl::setSceneManager(Ogre::SceneManager * scene_manager)
{
  ogre_scene_manager_ = scene_manager;
}

void RenderWindowImpl::setVisibilityMask(uint32_t mask)
{
  if (ogre_viewport_) {
    ogre_viewport_->setVisibilityMask(mask);
  } else {
    pending_visibility_masks_.emplace_back(mask);
  }
}


#if 0
void RenderWindowImpl::setOverlaysEnabled(bool overlays_enabled)
{
  overlays_enabled_ = overlays_enabled;
  viewport_->setOverlaysEnabled(overlays_enabled);
  if (right_viewport_) {
    right_viewport_->setOverlaysEnabled(overlays_enabled);
  }
}
#endif

void RenderWindowImpl::setBackgroundColor(Ogre::ColourValue background_color)
{
  background_color_ = background_color;
  ogre_viewport_->setBackgroundColour(background_color);
#if 0
  if (ogre_right_viewport_) {
    ogre_right_viewport_->setBackgroundColour(background_color);
  }
#endif
}

void RenderWindowImpl::setCameraAspectRatio()
{
  // auto width = parent_->width();
  auto width = parent_->width() ? parent_->width() : 100;
  // auto height = parent_->height();
  auto height = parent_->height() ? parent_->height() : 100;
  if (ogre_camera_) {
    ogre_camera_->setAspectRatio(Ogre::Real(width) / Ogre::Real(height));
    // if (right_ogre_camera_) {
    //   right_ogre_camera_->setAspectRatio(Ogre::Real(width()) / Ogre::Real(height()));
    // }

    if (ogre_camera_->getProjectionType() == Ogre::PT_ORTHOGRAPHIC) {
      Ogre::Matrix4 proj = buildScaledOrthoMatrix(
        -width / ortho_scale_ / 2, width / ortho_scale_ / 2,
        -height / ortho_scale_ / 2, height / ortho_scale_ / 2,
        ogre_camera_->getNearClipDistance(), ogre_camera_->getFarClipDistance());
      ogre_camera_->setCustomProjectionMatrix(true, proj);
    }
  }
}

#if 0
void RenderWindowImpl::setOrthoScale(float scale)
{
  ortho_scale_ = scale;

  setCameraAspectRatio();
}
#endif

#if 0
void RenderWindowImpl::setPreRenderCallback(boost::function<void()> func)
{
  pre_render_callback_ = func;
}
#endif

#if 0
void RenderWindowImpl::setPostRenderCallback(boost::function<void()> func)
{
  post_render_callback_ = func;
}
#endif

//------------------------------------------------------------------------------
#if 0
void RenderWindowImpl::paintEvent(QPaintEvent * e)
{
  if (auto_render_ && render_window_) {
    if (pre_render_callback_) {
      pre_render_callback_();
    }

    if (ogre_root_->_fireFrameStarted()) {
#if (OGRE_VERSION_MAJOR >= 1 && OGRE_VERSION_MINOR >= 6)
      ogre_root_->_fireFrameRenderingQueued();
#endif

      render_window_->update();

      ogre_root_->_fireFrameEnded();
    }

    if (post_render_callback_) {
      post_render_callback_();
    }
  }
}
#endif

}  // namespace rviz_rendering
