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

#include "render_window_impl.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#   pragma clang diagnostic ignored "-Wextra-semi"
# else
#  pragma GCC diagnostic ignored "-Wpedantic"
# endif
#endif

#include <OgreCamera.h>
#include <OgreEntity.h>
#include <OgreMaterialManager.h>
#include <OgreSceneNode.h>
#include <OgreTechnique.h>
#include <OgreTextureManager.h>
#include <OgreViewport.h>
#include <OgreWindowEventUtilities.h>

#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include <QtWidgets/QtWidgets>

#include <rviz_rendering/logging.hpp>

#include "grid.h"

namespace rviz_rendering
{

RenderWindowImpl::RenderWindowImpl(QWindow * parent)
: parent_(parent),
  render_system_(nullptr),
  ogre_render_window_(nullptr),
  ogre_scene_manager_(nullptr),
  animating_(false)
{}

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
  Ogre::WindowEventUtilities::messagePump();
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
  // printf("in RenderWindowImpl::renderLater()\n");
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
  // printf("in RenderWindowImpl::renderNow()\n");
  if (!parent_->isExposed()) {
    return;
  }

  if (!render_system_) {
    printf("in RenderWindowImpl::renderNow() -> initialize()\n");
    this->initialize();
  }

  this->render();

  if (animating_) {
    printf("in RenderWindowImpl::renderNow() -> renderLater() because of animating_\n");
    this->renderLater();
  }
}

void
createScene(Ogre::SceneManager * ogre_scene_manager)
{
  printf("in RenderWindowImpl::createScene()\n");
  /*
  Example scene
  Derive this class for your own purpose and overwite this function to have a
  working Ogre widget with your own content.
  */

  QColor color = Qt::gray;
  new rviz_rendering::Grid(ogre_scene_manager, nullptr,
    Grid::Lines,
    10,
    1.0f,
    0.03f,
    Ogre::ColourValue(color.redF(), color.greenF(), color.blueF(), color.alphaF() ));
  ogre_scene_manager->setAmbientLight(Ogre::ColourValue(1.0f, 0.0f, 0.0f));

  Ogre::Entity * sphereMesh =
    ogre_scene_manager->createEntity("mySphere", Ogre::SceneManager::PT_SPHERE);

  Ogre::SceneNode * childSceneNode =
    ogre_scene_manager->getRootSceneNode()->createChildSceneNode();

  childSceneNode->attachObject(sphereMesh);

  Ogre::MaterialPtr sphereMaterial = Ogre::MaterialManager::getSingleton().create(
    "SphereMaterial",
    Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
    true);

  sphereMaterial->getTechnique(0)->getPass(0)->setAmbient(0.1f, 0.1f, 0.1f);
  sphereMaterial->getTechnique(0)->getPass(0)->setDiffuse(0.2f, 0.2f, 0.2f, 1.0f);
  sphereMaterial->getTechnique(0)->getPass(0)->setSpecular(0.9f, 0.9f, 0.9f, 1.0f);
  // sphereMaterial->setAmbient(0.2f, 0.2f, 0.5f);
  // sphereMaterial->setSelfIllumination(0.2f, 0.2f, 0.1f);

  sphereMesh->setMaterialName("SphereMaterial");
  childSceneNode->setPosition(Ogre::Vector3(0.0f, 0.0f, 0.0f));
  childSceneNode->setScale(Ogre::Vector3(0.01f, 0.01f, 0.01f));  // Radius, in theory.

  Ogre::Light * light = ogre_scene_manager->createLight("MainLight");
  light->setPosition(20.0f, 80.0f, 50.0f);
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
    RVIZ_RENDERING_LOG_ERROR("Ogre::Root * is unexpectedly nullptr");
  }

  ogre_scene_manager_ = ogre_root->createSceneManager(Ogre::ST_GENERIC);

  ogre_camera_ = ogre_scene_manager_->createCamera("MainCamera");
  ogre_camera_->setNearClipDistance(0.1f);
  ogre_camera_->setFarClipDistance(200.0f);

  auto camera_node_ = ogre_scene_manager_->getRootSceneNode()->createChildSceneNode();
  ogre_camera_->setPosition(Ogre::Vector3(0.0f, 10.0f, 10.0f));
  ogre_camera_->lookAt(Ogre::Vector3(0.0f, 0.0f, 0.0f));
  camera_node_->attachObject(ogre_camera_);

  Ogre::Viewport * pViewPort = ogre_render_window_->addViewport(ogre_camera_);
  auto bg_color = Ogre::ColourValue(0.9f, 0.9f, 0.9f);
  pViewPort->setBackgroundColour(bg_color);

  ogre_camera_->setAspectRatio(
    Ogre::Real(ogre_render_window_->getWidth()) / Ogre::Real(ogre_render_window_->getHeight()));
  ogre_camera_->setAutoAspectRatio(true);

  Ogre::TextureManager::getSingleton().setDefaultNumMipmaps(5);
  Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();

  createScene(ogre_scene_manager_);
}

void
RenderWindowImpl::resize(size_t width, size_t height)
{
  printf("in RenderWindowImpl::resize(size_t %zu, size_t %zu)\n", width, height);
  if (ogre_render_window_) {
    ogre_render_window_->resize(width, height);
  }
  this->renderLater();
}

}  // namespace rviz_rendering
