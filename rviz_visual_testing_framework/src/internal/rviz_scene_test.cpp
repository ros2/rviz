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

#include "rviz_visual_testing_framework/internal/rviz_scene_test.hpp"

#include <iostream>
#include <memory>
#include <string>
#include <thread>

RvizTestScene::RvizTestScene(
  rviz_common::VisualizerApp * vapp,
  Ogre::Vector3 pose,
  Ogre::Vector3 look_at,
  std::shared_ptr<Executor> executor)
: render_window_(nullptr),
  manager_(nullptr),
  camera_(nullptr),
  cam_pose_(pose),
  cam_look_at_vector_(look_at),
  visualizer_app_(vapp),
  executor_(executor)
{
  setUp();
}

void RvizTestScene::setUp()
{
  render_window_ = visualizer_app_->getRenderWindow();
  manager_ = rviz_rendering::RenderWindowOgreAdapter::getSceneManager(render_window_);
}

void RvizTestScene::setUpCamera()
{
  static int camera_counter = 0;
  std::string cam_name = "TestCamera(" + std::to_string(++camera_counter) + ")";
  camera_ = manager_->createCamera(cam_name);

  camera_->setNearClipDistance(0.1f);
  camera_->setFarClipDistance(400.0f);

  cam_node_ = manager_->getRootSceneNode()->createChildSceneNode();
  cam_node_->attachObject(camera_);
  cam_node_->setPosition(cam_pose_);
  cam_node_->lookAt(cam_look_at_vector_, Ogre::Node::TS_WORLD);
}

void RvizTestScene::installCamera()
{
  rviz_rendering::RenderWindowOgreAdapter::setOgreCamera(render_window_, camera_);
}

void RvizTestScene::takeReferenceShot(Ogre::String name)
{
  takeScreenShot(name + "_ref.png");
}

void RvizTestScene::takeTestShot(Ogre::String name)
{
  takeScreenShot(name + ".png");
}

void RvizTestScene::takeScreenShot(Ogre::String name)
{
  executor_->queueAction(
    [this, name] {
      render_window_->captureScreenShot(name);
    }
  );
}

void RvizTestScene::setCamPose(Ogre::Vector3 pose)
{
  cam_node_->setPosition(pose);
}

void RvizTestScene::setLookAt(Ogre::Vector3 look_at)
{
  cam_node_->lookAt(look_at, Ogre::Node::TS_WORLD);
}
