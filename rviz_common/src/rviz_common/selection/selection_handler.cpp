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

#include "rviz_common/selection/selection_handler.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

#ifdef _WIN32
#pragma warning(push)
#pragma warning(disable : 4996)
#include <OgreEntity.h>
#pragma warning(pop)
#else
#include <OgreEntity.h>
#endif
#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreSubEntity.h>
#include <OgreWireBoundingBox.h>

#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include <cassert>
#include <string>
#include <utility>

#include "rviz_common/logging.hpp"

#include "rviz_common/properties/property.hpp"
#include "rviz_common/selection/selection_manager.hpp"
#include "../visualization_manager.hpp"

namespace rviz_common
{
namespace selection
{

using rviz_common::properties::Property;

SelectionHandler::SelectionHandler(DisplayContext * context)
: context_(context),
  listener_(new Listener(this))
{
  pick_handle_ = context_->getSelectionManager()->createHandle();
  context_->getSelectionManager()->addObject(pick_handle_, this);
}

SelectionHandler::~SelectionHandler()
{
  for (const auto & object : tracked_objects_) {
    object->setListener(nullptr);
  }

  while (!boxes_.empty()) {
    destroyBox(boxes_.begin()->first);
  }
  if (context_->getSelectionManager()) {
    context_->getSelectionManager()->removeObject(pick_handle_);
  }
}

void SelectionHandler::preRenderPass(uint32_t pass)
{
  Q_UNUSED(pass);
  setBoxVisibility(false);
}

void SelectionHandler::postRenderPass(uint32_t pass)
{
  Q_UNUSED(pass);
  setBoxVisibility(true);
}

void SelectionHandler::setBoxVisibility(bool visible)
{
  for (const auto & handle_to_box : boxes_) {
    handle_to_box.second.second->setVisible(visible);
  }
}

void SelectionHandler::addTrackedObjects(Ogre::SceneNode * node)
{
  if (!node) {
    return;
  }
  // Loop over all objects attached to this node.
  Ogre::SceneNode::ObjectIterator obj_it = node->getAttachedObjectIterator();
  while (obj_it.hasMoreElements() ) {
    Ogre::MovableObject * obj = obj_it.getNext();
    addTrackedObject(obj);
  }
  // Loop over and recurse into all child nodes.
  Ogre::SceneNode::ChildNodeIterator child_it = node->getChildIterator();
  while (child_it.hasMoreElements() ) {
    auto child = dynamic_cast<Ogre::SceneNode *>( child_it.getNext() );
    addTrackedObjects(child);
  }
}

void SelectionHandler::addTrackedObject(Ogre::MovableObject * object)
{
  tracked_objects_.insert(object);
  object->setListener(listener_.get());

  SelectionManager::setPickHandle(pick_handle_, object);
}

void SelectionHandler::removeTrackedObject(Ogre::MovableObject * object)
{
  tracked_objects_.erase(object);
  object->setListener(nullptr);

  updateTrackedBoxes();
}

void SelectionHandler::updateTrackedBoxes()
{
  for (const auto & handle_to_box : boxes_) {
    V_AABB aabbs;
    Picked p(handle_to_box.first.first);
    p.extra_handles.insert(handle_to_box.first.second);
    getAABBs(Picked(handle_to_box.first.first), aabbs);

    if (!aabbs.empty()) {
      Ogre::AxisAlignedBox combined;
      for (const auto & aabb : aabbs) {
        combined.merge(aabb);
      }

      createBox(std::make_pair(p.handle, handle_to_box.first.second), combined, "RVIZ/Cyan");
    }
  }
}

void SelectionHandler::createProperties(const Picked & obj, Property * parent_property)
{
  Q_UNUSED(obj);
  Q_UNUSED(parent_property);
}

void SelectionHandler::getAABBs(const Picked & obj, V_AABB & aabbs)
{
  Q_UNUSED(obj);
  for (const auto & tracked_object : tracked_objects_) {
    aabbs.push_back(tracked_object->getWorldBoundingBox());
  }
}

void SelectionHandler::destroyProperties(const Picked & obj, Property * parent_property)
{
  Q_UNUSED(obj);
  Q_UNUSED(parent_property);
  for (int i = 0; i < properties_.size(); i++) {
    delete properties_.at(i);
  }
  properties_.clear();
}

void SelectionHandler::updateProperties()
{}

bool SelectionHandler::needsAdditionalRenderPass(uint32_t pass)
{
  Q_UNUSED(pass);
  return false;
}

void SelectionHandler::createBox(
  const std::pair<CollObjectHandle, uint64_t> & handles,
  const Ogre::AxisAlignedBox & aabb,
  const std::string & material_name)
{
  Ogre::WireBoundingBox * box = nullptr;
  Ogre::SceneNode * node = nullptr;

  auto handle_to_box_iterator = boxes_.find(handles);
  if (handle_to_box_iterator == boxes_.end()) {
    Ogre::SceneManager * scene_manager = context_->getSceneManager();
    node = scene_manager->getRootSceneNode()->createChildSceneNode();
    box = new Ogre::WireBoundingBox;

    bool inserted = boxes_.insert(std::make_pair(handles, std::make_pair(node, box))).second;
    (void) inserted;
    assert(inserted);
  } else {
    node = handle_to_box_iterator->second.first;
    box = handle_to_box_iterator->second.second;
  }

  auto material = Ogre::MaterialManager::getSingleton().getByName(material_name);
  if (!material) {
    RVIZ_COMMON_LOG_ERROR_STREAM("failed to load material: " << material_name);
    return;
  }

  box->setMaterial(material);
  box->setupBoundingBox(aabb);
  node->detachAllObjects();
  node->attachObject(box);
}

void SelectionHandler::destroyBox(const std::pair<CollObjectHandle, uint64_t> & handles)
{
  auto handle_to_box_iterator = boxes_.find(handles);
  if (handle_to_box_iterator != boxes_.end()) {
    Ogre::SceneNode * node = handle_to_box_iterator->second.first;
    Ogre::WireBoundingBox * box = handle_to_box_iterator->second.second;

    node->detachAllObjects();
    node->getParentSceneNode()->removeAndDestroyChild(node->getName());

    delete box;
    box = nullptr;
    boxes_.erase(handle_to_box_iterator);
  }
}

void SelectionHandler::onSelect(const Picked & obj)
{
  V_AABB aabbs;
  getAABBs(obj, aabbs);

  if (!aabbs.empty()) {
    Ogre::AxisAlignedBox combined;
    for (const auto & aabb : aabbs) {
      combined.merge(aabb);
    }

    createBox(std::make_pair(obj.handle, 0ULL), combined, "RVIZ/Cyan");
  }
}

void SelectionHandler::onDeselect(const Picked & obj)
{
  destroyBox(std::make_pair(obj.handle, 0ULL));
}

void SelectionHandler::setInteractiveObject(InteractiveObjectWPtr object)
{
  interactive_object_ = object;
}

InteractiveObjectWPtr SelectionHandler::getInteractiveObject()
{
  return interactive_object_;
}

CollObjectHandle SelectionHandler::getHandle() const
{
  return pick_handle_;
}

SelectionHandler::Listener::Listener(SelectionHandler * handler)
: handler_(handler)
{}

void SelectionHandler::Listener::objectMoved(Ogre::MovableObject * object)
{
  Q_UNUSED(object);
  handler_->updateTrackedBoxes();
}

void SelectionHandler::Listener::objectDestroyed(Ogre::MovableObject * object)
{
  handler_->removeTrackedObject(object);
}

}  // namespace selection
}  // namespace rviz_common
