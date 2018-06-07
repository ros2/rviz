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

#ifndef RVIZ_COMMON__INTERACTION__SELECTION_HANDLER_HPP_
#define RVIZ_COMMON__INTERACTION__SELECTION_HANDLER_HPP_

#include <map>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <OgreMovableObject.h>

#include "rviz_common/interaction/forwards.hpp"
#include "rviz_common/interactive_object.hpp"
#include "rviz_common/properties/property.hpp"
#include "rviz_common/viewport_mouse_event.hpp"
#include "rviz_common/visibility_control.hpp"


namespace Ogre
{
class WireBoundingBox;
class SceneNode;
class MovableObject;
}  // namespace Ogre

namespace rviz_common
{

class DisplayContext;
class ViewportMouseEvent;

namespace interaction
{

using V_AABB = std::vector<Ogre::AxisAlignedBox>;

template<class T>
std::weak_ptr<T> weak_from_this(T * pointer)
{
  return pointer->shared_from_this();
}

/// Use this function to create a SelectionHandler
/**
 * This function creates a shared_ptr for any SelectionHandler, registering a handle with the
 * HandlerManager.
 *
 * Note: When migrating from rviz to rviz2, use this function to create a SelectionHandler
 * instead of constructors.
 *
 * @tparam T template type of SelectionHandler to be created
 * @tparam Args placeholder types for constructor parameter types
 * @param arguments arguments used in the constructor of the SelectionHandler to be created
 * @return
 */
template<typename T, typename ... Args>
std::shared_ptr<T> createSelectionHandler(Args ... arguments)
{
  auto selection_handler = std::shared_ptr<T>(new T(arguments ...));
  selection_handler->registerHandle();
  return selection_handler;
}

class RVIZ_COMMON_PUBLIC SelectionHandler : public std::enable_shared_from_this<SelectionHandler>
{
public:
  virtual ~SelectionHandler();

  void addTrackedObjects(Ogre::SceneNode * node);
  void addTrackedObject(Ogre::MovableObject * object);
  void removeTrackedObject(Ogre::MovableObject * object);

  virtual void updateTrackedBoxes();

  /// Override to create properties of the given picked object(s).
  /**
   * Top-level properties created here should be added to properties_ so they
   * will be automatically deleted by deleteProperties().
   *
   * This base implementation does nothing.
   */
  virtual void createProperties(
    const Picked & obj, rviz_common::properties::Property * parent_property);

  /// Destroy all properties for the given picked object(s).
  /**
   * This base implementation destroys all the properties in properties_.
   *
   * If createProperties() adds all the top-level properties to properties_,
   * there is no need to override this in a subclass.
   */
  virtual void destroyProperties(
    const Picked & obj, rviz_common::properties::Property * parent_property);

  /** @brief Override to update property values.
   *
   * updateProperties() is called on a timer to give selection
   * handlers a chance to update displayed property values.
   * Subclasses with properties that can change should implement this
   * to update the property values based on new information from the
   * selected object(s).
   *
   * This base implementation does nothing.
   */
  virtual void updateProperties();

  /// Override to indicate if an additional render pass is required.
  virtual bool needsAdditionalRenderPass(uint32_t pass);

  /// Override to hook before a render pass.
  virtual void preRenderPass(uint32_t pass);

  /// Override to hook after a render pass.
  virtual void postRenderPass(uint32_t pass);

  /// Get the AABBs.
  virtual V_AABB getAABBs(const Picked & obj);

  /// Override to get called on selection.
  virtual void onSelect(const Picked & obj);

  /// Override to get called on deselection.
  virtual void onDeselect(const Picked & obj);

  /// Set an object to listen to mouse events and other interaction calls.
  /**
   * Events occur during use of the 'interact' tool.
   */
  virtual void setInteractiveObject(InteractiveObjectWPtr object);

  /// Get the object to listen to mouse events and other interaction calls.
  /**
   * Returns a weak_ptr to the object, which may or may not
   * point to something.  Do not lock() the result and hold it for
   * long periods because it may cause something visual to stick
   * around after it was meant to be destroyed.
   */
  virtual InteractiveObjectWPtr getInteractiveObject();

  /// Get CollObjectHandle.
  CollObjectHandle getHandle() const;

  struct Handles
  {
    Handles(CollObjectHandle _handle, uint64_t _extra_handle)
    : handle(_handle), extra_handle(_extra_handle) {}

    bool operator==(const Handles & rhs) const
    {
      return handle == rhs.handle && extra_handle == rhs.extra_handle;
    }

    bool operator<(const Handles & rhs) const
    {
      if (handle < rhs.handle) {
        return true;
      } else if (handle > rhs.handle) {
        return false;
      } else if (extra_handle < rhs.extra_handle) {
        return true;
      }
      return false;
    }

    bool operator<=(const Handles & rhs) const
    {
      return *this == rhs || *this < rhs;
    }

    CollObjectHandle handle;
    uint64_t extra_handle;
  };

  struct SelectionBox
  {
    SelectionBox(Ogre::SceneNode * _node, Ogre::WireBoundingBox * _box)
    : scene_node(_node), box(_box) {}

    Ogre::SceneNode * scene_node;
    Ogre::WireBoundingBox * box;
  };

protected:
  explicit SelectionHandler(DisplayContext * context);

  void registerHandle();

  /// Create or update a box for the given handle-int pair, with the box specified by aabb.
  void createBox(
    const Handles & handles,
    const Ogre::AxisAlignedBox & aabb,
    const std::string & material_name);

  /// Destroy the box associated with the given handle-int pair, if there is one.
  void destroyBox(const Handles & handles);

  void setBoxVisibility(bool visible);

  QList<rviz_common::properties::Property *> properties_;

  using M_HandleToBox = std::map<Handles, SelectionBox>;
  M_HandleToBox boxes_;

  DisplayContext * context_;

  using S_Movable = std::set<Ogre::MovableObject *>;
  S_Movable tracked_objects_;

  class Listener : public Ogre::MovableObject::Listener
  {
public:
    // TODO(wjwwood): uncrustify doesn't handle this indentation correctly.
    explicit Listener(SelectionHandler * handler);

    void objectMoved(Ogre::MovableObject * object) override;

    void objectDestroyed(Ogre::MovableObject * object) override;

    SelectionHandler * handler_;
  };

  using ListenerPtr = std::shared_ptr<Listener>;
  ListenerPtr listener_;

  InteractiveObjectWPtr interactive_object_;

private:
  // pick_handle_ must never be changed, otherwise the destructor will
  // call removeObject() with the wrong handle.  Use getHandle() to
  // access the value.
  CollObjectHandle pick_handle_;

  friend class SelectionManager;
  template<typename T, typename ... Args>
  friend typename std::shared_ptr<T>  // typename is used only to make uncrustify happy
  rviz_common::interaction::createSelectionHandler(Args ... arguments);
};

using SelectionHandlerPtr = std::shared_ptr<SelectionHandler>;
using SelectionHandlerWeakPtr = std::weak_ptr<SelectionHandler>;
using V_SelectionHandler = std::vector<SelectionHandlerPtr>;
using S_SelectionHandler = std::set<SelectionHandlerPtr>;

}  // namespace interaction
}  // namespace rviz_common

#endif  // RVIZ_COMMON__INTERACTION__SELECTION_HANDLER_HPP_
