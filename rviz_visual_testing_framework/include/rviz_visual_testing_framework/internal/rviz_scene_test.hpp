/*
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
 *     * Neither the name of the copyright holder nor the names of its contributors
 *       may be used to endorse or promote products derived from
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

#ifndef RVIZ_VISUAL_TESTING_FRAMEWORK__INTERNAL__RVIZ_SCENE_TEST_HPP_
#define RVIZ_VISUAL_TESTING_FRAMEWORK__INTERNAL__RVIZ_SCENE_TEST_HPP_

#include <memory>

#ifdef _WIN32
# pragma warning(push)
# pragma warning(disable : 4996)
#endif
#include <Ogre.h>
#include <OgreSceneManager.h>
#include <OgreCamera.h>
#ifdef _WIN32
# pragma warning(pop)
#endif

#include "rviz_rendering/render_window.hpp"
#include "rviz_common/visualizer_app.hpp"

#include "rviz_visual_testing_framework/internal/executor.hpp"

class RvizTestScene
{
public:
  RvizTestScene(
    rviz_common::VisualizerApp * vapp,
    Ogre::Vector3 pose,
    Ogre::Vector3 look_at,
    std::shared_ptr<Executor> executor);

  void takeReferenceShot(Ogre::String name);
  void takeTestShot(Ogre::String name);
  void setCamPose(Ogre::Vector3 pose);
  void setLookAt(Ogre::Vector3 look_at);
  void setUpCamera();
  void installCamera();

private:
  void setUp();
  void takeScreenShot(Ogre::String name);

  rviz_rendering::RenderWindow * render_window_;
  Ogre::SceneManager * manager_;
  Ogre::Camera * camera_;
  Ogre::SceneNode * cam_node_;
  Ogre::Vector3 cam_pose_;
  Ogre::Vector3 cam_look_at_vector_;
  rviz_common::VisualizerApp * visualizer_app_;
  std::shared_ptr<Executor> executor_;
};

#endif  // RVIZ_VISUAL_TESTING_FRAMEWORK__INTERNAL__RVIZ_SCENE_TEST_HPP_
