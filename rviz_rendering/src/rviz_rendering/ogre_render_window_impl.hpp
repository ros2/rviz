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

#ifndef RVIZ_RENDERING__OGRE_RENDER_WINDOW_IMPL_HPP_
#define RVIZ_RENDERING__OGRE_RENDER_WINDOW_IMPL_HPP_

#include <functional>
#include <vector>

#include <OgreRenderTargetListener.h>
#include <OgreRenderWindow.h>
#include <OgreSceneManager.h>

#include <QEvent>  // NOLINT: cpplint cannot handle include order here
#include <QObject>  // NOLINT: cpplint cannot handle include order here
#include <QWindow>  // NOLINT: cpplint cannot handle include order here

#include "rviz_rendering/render_system.hpp"

namespace Ogre
{

class Root;
class RenderWindow;
class Viewport;
class Camera;

}  // namespace Ogre

namespace rviz_rendering
{

// TODO(wjwwood): rename this class to OgreRenderWindowImpl and have it inherit
//                from a pure-virtual abstract base class called RenderWindowImpl.
/// Implementation for the rviz_rendering::RenderWindow class that uses Ogre.
/**
 * Based on the QtOgreRenderWindow from previous versions of rviz and new
 * Ogre/Qt5 integration recommendationd.
 */
class RenderWindowImpl
{
public:
  explicit RenderWindowImpl(QWindow * parent);

  virtual ~RenderWindowImpl();

  void
  screenShot(Ogre::String imageName);

  void
  initialize();

  void
  render();

  void
  renderLater();

  void
  renderNow();

  void
  resize(size_t width, size_t height);

  using setupSceneCallback = std::function<void (Ogre::SceneNode *)>;
  void
  setupSceneAfterInit(setupSceneCallback setup_scene_callback);

#if 0
  /**
   * Set a callback which is called before each render
   * @param func The callback functor
   */
  virtual void setPreRenderCallback(boost::function<void()> func);
  /**
     * Set a callback which is called after each render
     * @param func The callback functor
     */
  virtual void setPostRenderCallback(boost::function<void()> func);
#endif

  /// Get the associated Ogre viewport.
  /**
   * If this is called before QWidget::show() on this widget, it will fail an
   * assertion.
   * Several functions of Ogre::Viewport are duplicated in this class
   * which can be called before QWidget::show(), and their effects are
   * propagated to the viewport when it is created.
   */
  Ogre::Viewport * getViewport() const;

  /// Set the camera associated with this render window's viewport.
  void setCamera(Ogre::Camera * camera);

  /// Get the camera.
  Ogre::Camera * getCamera() const;

  /// Get the main directional light.
  Ogre::Light * getDirectionalLight() const;

  /// Get the Ogre scene manager.
  Ogre::SceneManager * getSceneManager() const;

  void setSceneManager(Ogre::SceneManager * scene_manager);

  void addListener(Ogre::RenderTargetListener * listener);
  void removeListener(Ogre::RenderTargetListener * listener);
#if 0
  /**
   * \brief Set the scale of the orthographic window.  Only valid for an orthographic camera.
   * @param scale The scale
   */
  void setOrthoScale(float scale);

  /** \brief Enable or disable stereo rendering
   * If stereo is not supported this is ignored.
   * @return the old setting (whether stereo was enabled before)
   */
  bool enableStereo(bool enable);

  /** \brief Prepare to render in stereo if enabled and supported. */
  void setupStereo();

  void setAutoRender(bool auto_render) {auto_render_ = auto_render;}

  ////// Functions mimicked from Ogre::Viewport to satisfy timing of
  ////// after-constructor creation of Ogre::RenderWindow.
  void setOverlaysEnabled(bool overlays_enabled);
#endif
  void setBackgroundColor(Ogre::ColourValue color);

  void setVisibilityMask(uint32_t mask);

protected:
#if 0
  virtual void paintEvent(QPaintEvent * e);
  virtual void resizeEvent(QResizeEvent * event);

  // When stereo is enabled, these are called before/after rendering each
  // viewport.
  virtual void preViewportUpdate(const Ogre::RenderTargetViewportEvent & evt);
  virtual void postViewportUpdate(const Ogre::RenderTargetViewportEvent & evt);
#endif

  /// Set the aspect ratio on the camera.
  void setCameraAspectRatio();

#if 0
  /**
   * prepare a viewport's camera for stereo rendering.
   * This should only be called from StereoRenderTargetListener
   */
  void prepareStereoViewport(Ogre::Viewport *);
#endif

  QWindow * parent_;
  RenderSystem * render_system_;
  Ogre::RenderWindow * ogre_render_window_;
  Ogre::FrameListener * ogre_frame_listener_;
  Ogre::SceneManager * ogre_scene_manager_;
  Ogre::Camera * ogre_camera_;
  Ogre::Light * ogre_directional_light_;

  bool animating_;

  Ogre::Viewport * ogre_viewport_;

  // std::function<void()> pre_render_callback_;  ///< Functor which is called before each render
  // std::function<void()> post_render_callback_;  ///< Functor which is called after each render

  float ortho_scale_;
  // bool auto_render_;

  // bool overlays_enabled_;
  Ogre::ColourValue background_color_;

  // // stereo rendering
  // bool stereo_enabled_;                         // true if we were asked to render stereo
  // bool rendering_stereo_;                       // true if we are actually rendering stereo
  // Ogre::Camera * left_camera_;
  // Ogre::Camera * right_camera_;
  // Ogre::Viewport * right_viewport_;

  setupSceneCallback setup_scene_callback_;
  std::vector<Ogre::RenderTargetListener *> pending_listeners_;
  std::vector<uint32_t> pending_visibility_masks_;
};

}  // namespace rviz_rendering

#endif  // RVIZ_RENDERING__OGRE_RENDER_WINDOW_IMPL_HPP_
