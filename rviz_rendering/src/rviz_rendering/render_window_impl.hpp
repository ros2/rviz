/*
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

#ifndef RVIZ_RENDERING__RENDER_WINDOW_IMPL_HPP_
#define RVIZ_RENDERING__RENDER_WINDOW_IMPL_HPP_

#include <QEvent>
#include <QObject>
#include <QWindow>

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# pragma GCC diagnostic ignored "-Wdeprecated-declarations"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wkeyword-macro"
#  pragma clang diagnostic ignored "-Wextra-semi"
# else
#  pragma GCC diagnostic ignored "-Wunused-but-set-parameter"
#  pragma GCC diagnostic ignored "-Wpedantic"
# endif
#endif

#include <OgreRenderWindow.h>
#include <OgreSceneManager.h>

#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "render_system.hpp"

namespace rviz_rendering
{

class RenderWindowImpl
{
public:
  explicit RenderWindowImpl(QWindow * parent);

  void
  render();

  void
  renderLater();

  void
  renderNow();

  void
  initialize();

  void
  resize(size_t width, size_t height);

protected:
  QWindow * parent_;
  RenderSystem * render_system_;
  Ogre::RenderWindow * ogre_render_window_;
  Ogre::FrameListener * ogre_frame_listener_;
  Ogre::SceneManager * ogre_scene_manager_;
  Ogre::Camera * ogre_camera_;
  bool animating_;
};

}  // namespace rviz_rendering

#endif  // RVIZ_RENDERING__RENDER_WINDOW_IMPL_HPP_
