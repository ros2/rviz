/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * Copyright (c) 2017, Open Source Robotics Foundation, Inc.
 * Copyright (c) 2017, Bosch Software Innovations GmbH.
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

#include "render_system.hpp"

#include <map>
#include <string>
#include <vector>

#ifdef __linux__

#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <GL/glx.h>

#endif

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

#include <OgreRenderWindow.h>

#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "ament_index_cpp/get_resource.hpp"
#include "ament_index_cpp/get_resources.hpp"
#include "rviz_rendering/logging.hpp"
#include "rviz_rendering/resource_config.hpp"

#include "string_helper.hpp"
#include "ogre_logging.hpp"

namespace rviz_rendering
{

Ogre::GLPlugin * RenderSystem::render_system_gl_plugin_ = nullptr;
RenderSystem * RenderSystem::instance_ = nullptr;
int RenderSystem::force_gl_version_ = 0;
bool RenderSystem::use_anti_aliasing_ = true;
bool RenderSystem::force_no_stereo_ = false;

RenderSystem *
RenderSystem::get()
{
  if (instance_ == 0) {
    rviz_rendering::OgreLogging::configureLogging();
    instance_ = new RenderSystem();
  }
  return instance_;
}

Ogre::Root *
RenderSystem::getOgreRoot()
{
  return ogre_root_;
}

int
RenderSystem::getGlVersion()
{
  return gl_version_;
}

int
RenderSystem::getGlslVersion()
{
  return glsl_version_;
}

void
RenderSystem::forceGlVersion(int version)
{
  force_gl_version_ = version;
  RVIZ_RENDERING_LOG_INFO_STREAM("Forcing OpenGl version " << version / 100.0 << ".");
}

void
RenderSystem::disableAntiAliasing()
{
  use_anti_aliasing_ = false;
  RVIZ_RENDERING_LOG_INFO("Disabling Anti-Aliasing");
}

bool
RenderSystem::isStereoSupported()
{
  return stereo_supported_;
}

void
RenderSystem::forceNoStereo()
{
  force_no_stereo_ = true;
  RVIZ_RENDERING_LOG_INFO("Forcing Stereo OFF");
}

RenderSystem::RenderSystem()
: dummy_window_id_(0), ogre_overlay_system_(nullptr), stereo_supported_(false)
{
  OgreLogging::configureLogging();

  setResourceDirectory();
  setupDummyWindowId();
  ogre_root_ = new Ogre::Root(get_resource_directory() + "/ogre_media/plugins.cfg");
#if ((OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR >= 9) || OGRE_VERSION_MAJOR >= 2)
  ogre_overlay_system_ = new Ogre::OverlaySystem();
#endif
  loadOgrePlugins();
  setupRenderSystem();
  ogre_root_->initialise(false);
  makeRenderWindow(dummy_window_id_, 1, 1);
  detectGlVersion();
  setupResources();
  Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();
}

void
RenderSystem::prepareOverlays(Ogre::SceneManager * scene_manager)
{
#if ((OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR >= 9) || OGRE_VERSION_MAJOR >= 2)
  if (ogre_overlay_system_) {
    scene_manager->addRenderQueueListener(ogre_overlay_system_);
  }
#endif
}

void
RenderSystem::setupDummyWindowId()
{
  dummy_window_id_ = 0;
#ifdef __linux__
  Display * display = XOpenDisplay(0);
  assert(display);

  int screen = DefaultScreen(display);

  int attribList[] = {GLX_RGBA, GLX_DOUBLEBUFFER, GLX_DEPTH_SIZE, 16,
    GLX_STENCIL_SIZE, 8, None};

  XVisualInfo * visual = glXChooseVisual(display, screen, attribList);

  dummy_window_id_ = XCreateSimpleWindow(display,
      RootWindow(display, screen),
      0, 0, 1, 1, 0, 0, 0);

  GLXContext context = glXCreateContext(display, visual, nullptr, 1);

  glXMakeCurrent(display, dummy_window_id_, context);
#endif
}

void
RenderSystem::loadOgrePlugins()
{
  // std::string plugin_prefix = get_ogre_plugin_directory() + "/";

  render_system_gl_plugin_ = new Ogre::GLPlugin();
  ogre_root_->installPlugin(render_system_gl_plugin_);

// #if __APPLE__
// ogre_root_->loadPlugin(plugin_prefix + "RenderSystem_GL");
// #else
// ogre_root_->loadPlugin(plugin_prefix + "RenderSystem_GL3Plus");
// #endif
// ogre_root_->loadPlugin(plugin_prefix + "Plugin_OctreeSceneManager");
// ogre_root_->loadPlugin(plugin_prefix + "Plugin_ParticleFX");
}

void
RenderSystem::detectGlVersion()
{
  if (force_gl_version_) {
    gl_version_ = force_gl_version_;
  } else {
    Ogre::RenderSystem * renderSys = ogre_root_->getRenderSystem();
    renderSys->createRenderSystemCapabilities();
    const Ogre::RenderSystemCapabilities * caps = renderSys->getCapabilities();
    int major = caps->getDriverVersion().major;
    int minor = caps->getDriverVersion().minor;
    gl_version_ = major * 100 + minor * 10;
  }

  switch (gl_version_) {
    case 200:
      glsl_version_ = 110;
      break;
    case 210:
      glsl_version_ = 120;
      break;
    case 300:
      glsl_version_ = 130;
      break;
    case 310:
      glsl_version_ = 140;
      break;
    case 320:
      glsl_version_ = 150;
      break;
    default:
      if (gl_version_ > 320) {
        glsl_version_ = gl_version_;
      } else {
        glsl_version_ = 0;
      }
      break;
  }
  RVIZ_RENDERING_LOG_INFO_STREAM(
    "OpenGl version: " << gl_version_ / 100.0 << " (GLSL " << glsl_version_ / 100.0 << ")"
  );
}

void
RenderSystem::setupRenderSystem()
{
  Ogre::RenderSystem * render_system;

  // Look for the OpenGL one, which we require.
  render_system = nullptr;
  std::string renderers_msg =
    "Available Renderers(" + std::to_string(ogre_root_->getAvailableRenderers().size()) + "): ";
  for (const auto renderer : ogre_root_->getAvailableRenderers()) {
    renderers_msg += renderer->getName() + ",";
  }
  RVIZ_RENDERING_LOG_DEBUG(renderers_msg.substr(0, renderers_msg.length() - 1));
  std::vector<std::string> preferred_renderer_list = {
    "OpenGL 3+",
    "OpenGL"
  };
  for (auto renderer_token : preferred_renderer_list) {
    for (const auto renderer : ogre_root_->getAvailableRenderers()) {
      if (renderer->getName().find(renderer_token) != Ogre::String::npos) {
        render_system = renderer;
        break;
      }
    }
  }

  if (render_system == nullptr) {
    throw std::runtime_error("Could not find the opengl rendering subsystem!");
  }

  // We operate in windowed mode by default.
  render_system->setConfigOption("Full Screen", "No");

  // Set the Full Screen Anti-Aliasing factor.
  if (use_anti_aliasing_) {
    render_system->setConfigOption("FSAA", "4");
  }

  ogre_root_->setRenderSystem(render_system);
}

void
RenderSystem::setResourceDirectory()
{
  std::string content;
  std::string prefix_path;
  ament_index_cpp::get_resource("packages", "rviz_rendering", content, &prefix_path);
  set_resource_directory(prefix_path + "/share/rviz_rendering");
}

void
RenderSystem::setupResources()
{
  std::string rviz_path = get_resource_directory();
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
    rviz_path + "/ogre_media", "FileSystem", "rviz_rendering");
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
    rviz_path + "/ogre_media/textures", "FileSystem", "rviz_rendering");
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
    rviz_path + "/ogre_media/fonts", "FileSystem", "rviz_rendering");
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
    rviz_path + "/ogre_media/fonts/liberation-sans", "FileSystem", "rviz_rendering");
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
    rviz_path + "/ogre_media/models", "FileSystem", "rviz_rendering");
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
    rviz_path + "/ogre_media/materials", "FileSystem", "rviz_rendering");
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
    rviz_path + "/ogre_media/materials/scripts", "FileSystem", "rviz_rendering");
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
    rviz_path + "/ogre_media/materials/glsl120", "FileSystem", "rviz_rendering");
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
    rviz_path + "/ogre_media/materials/glsl120/include", "FileSystem", "rviz_rendering");
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
    rviz_path + "/ogre_media/materials/glsl120/nogp", "FileSystem", "rviz_rendering");
  // Add resources that depend on a specific glsl version.
  // Unfortunately, Ogre doesn't have a notion of glsl versions so we can't go
  // the 'official' way of defining multiple schemes per material and let Ogre
  // decide which one to use.
  // TODO(wjwwood): figure out why includes don't work on 150
  // if (getGlslVersion() >= 150) {
  //   Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
  //     rviz_path + "/ogre_media/materials/glsl150", "FileSystem", "rviz_rendering");
  //   Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
  //     rviz_path + "/ogre_media/materials/scripts150", "FileSystem", "rviz_rendering");
  // } else if (getGlslVersion() >= 120) {
  if (getGlslVersion() >= 120) {
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
      rviz_path + "/ogre_media/materials/scripts120", "FileSystem", "rviz_rendering");
  } else {
    std::string s =
      "Your graphics driver does not support OpenGL 2.1. "
      "Please enable software rendering before running RViz "
      "(e.g. type 'export LIBGL_ALWAYS_SOFTWARE=1').";
    RVIZ_RENDERING_LOG_ERROR(s);
    throw std::runtime_error(s);
  }

  addAdditionalResourcesFromAmentIndex();
}

void RenderSystem::addAdditionalResourcesFromAmentIndex() const
{
  const std::string RVIZ_OGRE_MEDIA_RESOURCE_NAME = "rviz_ogre_media_exports";
  std::map<std::string,
    std::string> resource_locations = ament_index_cpp::get_resources(
    RVIZ_OGRE_MEDIA_RESOURCE_NAME);
  for (auto resource : resource_locations) {
    std::string content;
    std::string prefix_path;
    if (ament_index_cpp::get_resource(RVIZ_OGRE_MEDIA_RESOURCE_NAME, resource.first, content,
      &prefix_path))
    {
      std::vector<std::string> filenames =
        string_helper::splitStringIntoTrimmedItems(content, '\n');
      for (const auto & line : filenames) {
        std::string resource_path = prefix_path + "/share/" + line;
        if (!QDir(QString::fromStdString(resource_path)).exists()) {
          RVIZ_RENDERING_LOG_WARNING_STREAM("Could not find folder " << resource_path);
        }
        Ogre::ResourceGroupManager::getSingleton().addResourceLocation(resource_path, "FileSystem",
          "rviz_rendering");
      }
    }
  }
}

// On Intel graphics chips under X11, there sometimes comes a
// BadDrawable error during Ogre render window creation.  It is not
// consistent, happens sometimes but not always.  Underlying problem
// seems to be a driver bug.  My workaround here is to notice when
// that specific BadDrawable error happens on request 136 minor 3
// (which is what the problem looks like when it happens) and just try
// over and over again until it works (or until 100 failures, which
// makes it seem like it is a different bug).
static bool x_baddrawable_error = false;
#ifdef __linux__
static int (* old_error_handler)(Display *, XErrorEvent *);
int checkBadDrawable(Display * display, XErrorEvent * error)
{
  if (error->error_code == BadDrawable &&
    error->request_code == 136 &&
    error->minor_code == 3)
  {
    x_baddrawable_error = true;
    return 0;
  } else {
    // If the error does not exactly match the one from the driver bug,
    // handle it the normal way so we see it.
    return old_error_handler(display, error);
  }
}
#endif  // __linux__

Ogre::RenderWindow *
RenderSystem::makeRenderWindow(
  WindowIDType window_id,
  unsigned int width,
  unsigned int height,
  double pixel_ratio)
{
  static int window_counter = 0;  // Every RenderWindow needs a unique name, oy.

  Ogre::NameValuePairList params;
  Ogre::RenderWindow * window = nullptr;

  params["currentGLContext"] = Ogre::String("false");

  params["externalWindowHandle"] = Ogre::StringConverter::toString(window_id);
  params["parentWindowHandle"] = Ogre::StringConverter::toString(window_id);

  // Scale rendering window correctly on Windows
  params["left"] = std::to_string(0);
  params["top"] = std::to_string(0);

  // Hide dummy window borders on Windows
  if (width <= 1) {
    params["border"] = "none";
  }

  // params["externalGLControl"] = Ogre::String("true");

  // Enable antialiasing
  if (use_anti_aliasing_) {
    params["FSAA"] = "4";
  }

// Set the macAPI for Ogre based on the Qt implementation
#if __APPLE__
  params["macAPI"] = "cocoa";
  params["macAPICocoaUseNSView"] = "true";
#endif
  params["contentScalingFactor"] = std::to_string(pixel_ratio);

  std::ostringstream stream;
  stream << "OgreWindow(" << window_counter++ << ")";


  // don't bother trying stereo if Ogre does not support it.
#if !OGRE_STEREO_ENABLE
  force_no_stereo_ = true;
#endif

  // attempt to create a stereo window
  bool is_stereo = false;
  if (!force_no_stereo_) {
    params["stereoMode"] = "Frame Sequential";
    window = tryMakeRenderWindow(stream.str(), width, height, &params, 100);
    params.erase("stereoMode");

    if (window) {
#if OGRE_STEREO_ENABLE
      is_stereo = window->isStereoEnabled();
#endif
      if (!is_stereo) {
        // Created a non-stereo window.  Discard it and try again (below)
        // without the stereo parameter.
        ogre_root_->detachRenderTarget(window);
        window->destroy();
        window = nullptr;
        stream << "x";
        is_stereo = false;
      }
    }
  }

  if (window == nullptr) {
    window = tryMakeRenderWindow(stream.str(), width, height, &params, 100);
  }

  if (window == nullptr) {
    const char * msg = "Unable to create the rendering window after 100 tries";
    RVIZ_RENDERING_LOG_ERROR(msg);
    throw std::runtime_error(msg);
  }

  // Hide dummy window immediately on Windows
  if (width <= 1) {
    window->setHidden(true);
  } else {
    window->setVisible(true);
  }

  stereo_supported_ = is_stereo;

  RVIZ_RENDERING_LOG_INFO_STREAM(
    "Stereo is " << (stereo_supported_ ? "SUPPORTED" : "NOT SUPPORTED"));

  return window;
}

Ogre::RenderWindow *
RenderSystem::tryMakeRenderWindow(
  const std::string & name,
  unsigned int width,
  unsigned int height,
  const Ogre::NameValuePairList * params,
  int max_attempts)
{
  Ogre::RenderWindow * window = nullptr;
  int attempts = 0;

#ifdef __linux___
  old_error_handler = XSetErrorHandler(&checkBadDrawable);
#endif

  while (window == nullptr && attempts++ < max_attempts) {
    try {
      window = ogre_root_->createRenderWindow(name, width, height, false, params);

      // If the driver bug happened, tell Ogre we are done with that
      // window and then try again.
      if (x_baddrawable_error) {
        ogre_root_->detachRenderTarget(window);
        window = nullptr;
        x_baddrawable_error = false;
      }
    } catch (const std::exception & ex) {
      RVIZ_RENDERING_LOG_ERROR_STREAM(
        "rviz::RenderSystem: error creating render window: " << ex.what());
      window = nullptr;
    }
  }

#ifdef __linux___
  XSetErrorHandler(old_error_handler);
#endif

  if (window && attempts > 1) {
    RVIZ_RENDERING_LOG_INFO_STREAM("Created render window after " << attempts << " attempts.");
  }

  return window;
}

}  // namespace rviz_rendering
