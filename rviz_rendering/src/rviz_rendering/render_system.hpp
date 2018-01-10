/*
 * Copyright (c) 2011, Willow Garage, Inc.
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

#ifndef RVIZ_RENDERING__RENDER_SYSTEM_HPP_
#define RVIZ_RENDERING__RENDER_SYSTEM_HPP_

#include <cstdint>
#include <string>

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Woverloaded-virtual"
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wextra-semi"
#  pragma clang diagnostic ignored "-Wkeyword-macro"
# else
#  pragma GCC diagnostic ignored "-Wunused-but-set-parameter"
#  pragma GCC diagnostic ignored "-Wpedantic"
# endif
#endif

#include <OgreRoot.h>
#include <OgreOverlaySystem.h>
#include <RenderSystems/GL/OgreGLPlugin.h>

#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include <QDir>  // NOLINT

#include "rviz_rendering/visibility_control.hpp"

namespace rviz_rendering
{

class RenderSystem
{
public:
#if defined(__APPLE__) || defined(_WIN32)
  typedef size_t WindowIDType;
#else
  typedef unsigned long WindowIDType;  // NOLINT: we need to use C longs here
#endif

  RVIZ_RENDERING_PUBLIC
  static
  RenderSystem *
  get();

  Ogre::RenderWindow *
  makeRenderWindow(
    WindowIDType window_id,
    unsigned int width,
    unsigned int height,
    double pixel_ratio = 1.0);

  Ogre::Root *
  getOgreRoot();

  // Prepare a scene_manager to render overlays.
  // Needed for Ogre >= 1.9 to use fonts; does nothing for prior versions.
  void
  prepareOverlays(Ogre::SceneManager * scene_manager);

  /// return OpenGl Version as integer, e.g. 320 for OpenGl 3.20
  int
  getGlVersion();

  /// return GLSL Version as integer, e.g. 150 for GLSL 1.50
  int
  getGlslVersion();

  /// Disables the use of Anti Aliasing
  static
  void
  disableAntiAliasing();

  /// Force to use the provided OpenGL version on startup
  static
  void
  forceGlVersion(int version);

  /// Disable stereo rendering even if supported in HW.
  static
  void
  forceNoStereo();

  /// True if we can render stereo on this device.
  bool
  isStereoSupported();

private:
  RenderSystem();

  void
  setupDummyWindowId();

  void
  loadOgrePlugins();

  // helper for makeRenderWindow()
  Ogre::RenderWindow *
  tryMakeRenderWindow(
    const std::string & name,
    unsigned int width,
    unsigned int height,
    const Ogre::NameValuePairList * params,
    int max_attempts);

  // Find and configure the render system.
  void
  setupRenderSystem();
  void
  setResourceDirectory();
  void
  setPluginDirectory();
  void
  setupResources();
  void
  addAdditionalResourcesFromAmentIndex() const;
  void
  detectGlVersion();

  static Ogre::GLPlugin * render_system_gl_plugin_;

  static RenderSystem * instance_;

  // ID for a dummy window of size 1x1, used to keep Ogre happy.
  WindowIDType dummy_window_id_;

  Ogre::Root * ogre_root_;
  Ogre::OverlaySystem * ogre_overlay_system_;

  int gl_version_;
  int glsl_version_;
  static bool use_anti_aliasing_;
  static int force_gl_version_;
  bool stereo_supported_;
  static bool force_no_stereo_;
};

}  // namespace rviz_rendering

#endif  // RVIZ_RENDERING__RENDER_SYSTEM_HPP_
