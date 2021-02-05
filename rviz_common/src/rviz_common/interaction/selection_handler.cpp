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

#include "rviz_common/interaction/selection_handler.hpp"

#include <OgreEntity.h>
#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreSubEntity.h>
#include <OgreWireBoundingBox.h>

#include <cassert>
#include <string>
#include <utility>

#include "rviz_common/logging.hpp"

#include "rviz_common/properties/property.hpp"
#include "rviz_common/interaction/selection_manager.hpp"
#include "rviz_common/interaction/handler_manager_iface.hpp"
#include "rviz_common/visualization_manager.hpp"

namespace rviz_common
{
namespace interaction
{

using rviz_common::properties::Property;

SelectionHandler::SelectionHandler(DisplayContext * context)
: context_(context),
  listener_(new Listener(this))
{}

SelectionHandler::~SelectionHandler()
{
  for (const auto & object : tracked_objects_) {
    object->setListener(nullptr);
  }

  while (!boxes_.empty()) {
    destroyBox(boxes_.begin()->first);
  }
  if (context_->getHandlerManager()) {
    context_->getHandlerManager()->removeHandler(pick_handle_);
  }
  for (int i = 0; i < properties_.size(); i++) {
    delete properties_.at(i);
  }
  properties_.clear();
}

void SelectionHandler::registerHandle()
{
  pick_handle_ = context_->getHandlerManager()->createHandle();
  context_->getHandlerManager()->addHandler(
    pick_handle_, rviz_common::interaction::weak_from_this(this));
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
    handle_to_box.second.box->setVisible(visible);
  }
}

void SelectionHandler::addTrackedObjects(Ogre::SceneNode * node)
{
  if (!node) {
    return;
  }
  // Loop over all objects attached to this node.
  auto objects = node->getAttachedObjects();
  for (const auto & object : objects) {
    addTrackedObject(object);
  }
  // Loop over and recurse into all child nodes.
  for (auto child_node : node->getChildren()) {
    auto child = dynamic_cast<Ogre::SceneNode *>(child_node);
    if (child != nullptr) {
      addTrackedObjects(child);
    }
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
    Picked picked(handle_to_box.first.handle);
    picked.extra_handles.insert(handle_to_box.first.extra_handle);
    auto aabbs = getAABBs(picked);

    if (!aabbs.empty()) {
      Ogre::AxisAlignedBox combined;
      for (const auto & aabb : aabbs) {
        combined.merge(aabb);
      }

      createBox(handle_to_box.first, combined, "RVIZ/Cyan");
    }
  }
}

void SelectionHandler::createProperties(const Picked & obj, Property * parent_property)
{
  Q_UNUSED(obj);
  Q_UNUSED(parent_property);
}

V_AABB SelectionHandler::getAABBs(const Picked & obj)
{
  Q_UNUSED(obj);
  V_AABB aabbs;

  /** with 'derive_world_bounding_box' set to 'true', the WorldBoundingBox is derived each time.
      setting it to 'false' may result in the wire box not properly following the tracked object,
      but would be less computationally expensive.
  */
  bool derive_world_bounding_box = true;
  for (const auto & tracked_object : tracked_objects_) {
    aabbs.push_back(tracked_object->getWorldBoundingBox(derive_world_bounding_box));
  }

  return aabbs;
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
  const Handles & handles,
  const Ogre::AxisAlignedBox & aabb,
  const std::string & material_name)
{
  Ogre::WireBoundingBox * box = nullptr;
  Ogre::SceneNode * node = nullptr;

  auto handle_to_box_iterator = boxes_.find(handles);
  if (handle_to_box_iterator == boxes_.end()) {
    auto scene_manager = context_->getSceneManager();
    node = scene_manager->getRootSceneNode()->createChildSceneNode();
    box = new Ogre::WireBoundingBox;

    bool inserted = boxes_.insert(std::make_pair(handles, SelectionBox(node, box))).second;
    (void) inserted;
    assert(inserted);
  } else {
    node = handle_to_box_iterator->second.scene_node;
    box = handle_to_box_iterator->second.box;
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

void SelectionHandler::destroyBox(const Handles & handles)
{
  auto handle_to_box_iterator = boxes_.find(handles);
  if (handle_to_box_iterator != boxes_.end()) {
    auto node = handle_to_box_iterator->second.scene_node;
    auto box = handle_to_box_iterator->second.box;

    node->detachAllObjects();
    node->getParentSceneNode()->removeAndDestroyChild(node);

    delete box;
    box = nullptr;
    boxes_.erase(handle_to_box_iterator);
  }
}

void SelectionHandler::onSelect(const Picked & obj)
{
  auto aabbs = getAABBs(obj);

  if (!aabbs.empty()) {
    Ogre::AxisAlignedBox combined;
    for (const auto & aabb : aabbs) {
      combined.merge(aabb);
    }

    createBox(Handles(obj.handle, 0ULL), combined, "RVIZ/Cyan");
  }
}

void SelectionHandler::onDeselect(const Picked & obj)
{
  destroyBox(Handles(obj.handle, 0ULL));
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

}  // namespace interaction
}  // namespace rviz_common
