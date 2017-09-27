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

#include "./selection_handler.hpp"

#include <cassert>

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

#include <OgreEntity.h>
#include <OgreManualObject.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreSubEntity.h>
#include <OgreWireBoundingBox.h>

#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "rviz_common/logging.hpp"

#include "rviz_common/properties/property.hpp"
#include "../selection/selection_manager.hpp"
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
  S_Movable::iterator it = tracked_objects_.begin();
  S_Movable::iterator end = tracked_objects_.end();
  for (; it != end; ++it) {
    Ogre::MovableObject * m = *it;
    m->setListener(0);
  }

  while (!boxes_.empty()) {
    destroyBox(boxes_.begin()->first);
  }
  context_->getSelectionManager()->removeObject(pick_handle_);
}

void SelectionHandler::preRenderPass(uint32_t pass)
{
  Q_UNUSED(pass);
  M_HandleToBox::iterator it = boxes_.begin();
  M_HandleToBox::iterator end = boxes_.end();
  for (; it != end; ++it) {
    Ogre::WireBoundingBox * box = it->second.second;
    box->setVisible(false);
  }
}

void SelectionHandler::postRenderPass(uint32_t pass)
{
  Q_UNUSED(pass);
  M_HandleToBox::iterator it = boxes_.begin();
  M_HandleToBox::iterator end = boxes_.end();
  for (; it != end; ++it) {
    Ogre::WireBoundingBox * box = it->second.second;
    box->setVisible(true);
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
    Ogre::SceneNode * child = dynamic_cast<Ogre::SceneNode *>( child_it.getNext() );
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
  object->setListener(0);

  updateTrackedBoxes();
}

void SelectionHandler::updateTrackedBoxes()
{
  M_HandleToBox::iterator it = boxes_.begin();
  M_HandleToBox::iterator end = boxes_.end();
  for (; it != end; ++it) {
    V_AABB aabbs;
    Picked p(it->first.first);
    p.extra_handles.insert(it->first.second);
    getAABBs(Picked(it->first.first), aabbs);

    if (!aabbs.empty()) {
      Ogre::AxisAlignedBox combined;
      V_AABB::iterator aabb_it = aabbs.begin();
      V_AABB::iterator aabb_end = aabbs.end();
      for (; aabb_it != aabb_end; ++aabb_it) {
        combined.merge(*aabb_it);
      }

      createBox(std::make_pair(p.handle, it->first.second), combined, "RVIZ/Cyan");
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
  S_Movable::iterator it = tracked_objects_.begin();
  S_Movable::iterator end = tracked_objects_.end();
  for (; it != end; ++it) {
    aabbs.push_back((*it)->getWorldBoundingBox());
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
{
}

bool SelectionHandler::needsAdditionalRenderPass(uint32_t pass)
{
  Q_UNUSED(pass);
  return false;
}

void SelectionHandler::createBox(const std::pair<CollObjectHandle, uint64_t> & handles,
  const Ogre::AxisAlignedBox & aabb,
  const std::string & material_name)
{
  Ogre::WireBoundingBox * box = 0;
  Ogre::SceneNode * node = 0;

  M_HandleToBox::iterator it = boxes_.find(handles);
  if (it == boxes_.end()) {
    Ogre::SceneManager * scene_manager = context_->getSceneManager();
    node = scene_manager->getRootSceneNode()->createChildSceneNode();
    box = new Ogre::WireBoundingBox;

    bool inserted = boxes_.insert(std::make_pair(handles, std::make_pair(node, box))).second;
    assert(inserted);
  } else {
    node = it->second.first;
    box = it->second.second;
  }

  box->setMaterial(material_name);

  box->setupBoundingBox(aabb);
  node->detachAllObjects();
  node->attachObject(box);
}

void SelectionHandler::destroyBox(const std::pair<CollObjectHandle, uint64_t> & handles)
{
  M_HandleToBox::iterator it = boxes_.find(handles);
  if (it != boxes_.end()) {
    Ogre::SceneNode * node = it->second.first;
    Ogre::WireBoundingBox * box = it->second.second;

    node->detachAllObjects();
    node->getParentSceneNode()->removeAndDestroyChild(node->getName());

    delete box;

    boxes_.erase(it);
  }
}

void SelectionHandler::onSelect(const Picked & obj)
{
  V_AABB aabbs;
  getAABBs(obj, aabbs);

  if (!aabbs.empty()) {
    Ogre::AxisAlignedBox combined;
    V_AABB::iterator it = aabbs.begin();
    V_AABB::iterator end = aabbs.end();
    for (; it != end; ++it) {
      combined.merge(*it);
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

}  // namespace selection
}  // namespace rviz_common
