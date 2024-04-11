/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * Copyright (c) 2017, Open Source Robotics Foundation, Inc.
 * Copyright (c) 2018, Bosch Software Innovations GmbH.
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

#ifndef RVIZ_COMMON__INTERACTION__HANDLER_MANAGER_IFACE_HPP_
#define RVIZ_COMMON__INTERACTION__HANDLER_MANAGER_IFACE_HPP_

#include <iterator>
#include <memory>
#include <mutex>
#include <unordered_map>

#include "rviz_common/interaction/handler_manager_listener.hpp"
#include "rviz_common/interaction/selection_handler.hpp"
#include "rviz_common/visibility_control.hpp"

namespace rviz_common
{

class DisplayContext;

namespace interaction
{

using M_ObjectHandleToSelectionHandler =
  std::unordered_map<CollObjectHandle, SelectionHandlerWeakPtr>;

/**
 * Light-weight container wrapper for M_ObjectHandleToSelectionHandler that allows iterating
 * directly over handlers.
 */
class RVIZ_COMMON_PUBLIC HandlerRange
{
public:
  class iterator
  {
public:
    using iterator_category = std::forward_iterator_tag;
    using value_type = SelectionHandlerWeakPtr;
    using difference_type = std::ptrdiff_t;
    using pointer = SelectionHandlerWeakPtr *;
    using reference = SelectionHandlerWeakPtr &;

    // TODO(anhosi) uncrustify does not handle this identation correctly
    reference operator*() const
    {
      return iterator_->second;
    }

    bool operator!=(iterator other)
    {
      return iterator_ != other.iterator_;
    }

    iterator & operator++()
    {
      ++iterator_;
      return *this;
    }

    iterator operator++(int)
    {
      return iterator(iterator_++);
    }

private:
    explicit iterator(M_ObjectHandleToSelectionHandler::iterator it)
    : iterator_(it) {}

    M_ObjectHandleToSelectionHandler::iterator iterator_;

    friend class HandlerRange;
  };

  explicit HandlerRange(M_ObjectHandleToSelectionHandler & handlers)
  : handlers_(handlers) {}

  iterator begin()
  {
    return iterator(handlers_.begin());
  }

  iterator end()
  {
    return iterator(handlers_.end());
  }

private:
  M_ObjectHandleToSelectionHandler & handlers_;
};


/**
 * \brief The HandlerManagerIface manages selection handlers
 *
 * It is mainly used by the SelectionManager
 *
 * The HandlerManager must be locked (using one of the lock methods) if exclusive access to the
 * handlers is necessary.
 */
class RVIZ_COMMON_PUBLIC HandlerManagerIface
{
public:
  virtual ~HandlerManagerIface() = default;

  /**
   * Registers a new handle-handler pair. Locks internally to guarantee exclusive access.
   * \param handle
   * \param handler
   */
  virtual void addHandler(CollObjectHandle handle, SelectionHandlerWeakPtr handler) = 0;

  /**
   * Removes a handle-handler pair identified by its handle. Locks internally to guarantee
   * exclusive access
   * \param handle
   */
  virtual void removeHandler(CollObjectHandle handle) = 0;

  /// obtains the handler for a handle
  virtual SelectionHandlerPtr getHandler(CollObjectHandle handle) = 0;

  /**
   * Locks the handlers container
   * \return the lock for the handlers (already locked)
   */
  virtual std::unique_lock<std::recursive_mutex> lock() = 0;

  /**
   * Returns a lock for the handlers container that is not yet locked
   * \param defer_lock std::defer_lock
   * \return the lock for the handlers (not yet locked)
   */
  virtual std::unique_lock<std::recursive_mutex> lock(std::defer_lock_t defer_lock) = 0;

  /**
   * Registers a listener that is notified for every handle that is removed
   * \param listener
   */
  virtual void addListener(HandlerManagerListener * listener) = 0;

  /**
   * Removes a listeners.
   * \param listener
   */
  virtual void removeListener(HandlerManagerListener * listener) = 0;

  /**
   * Creates a new unique handle.
   * \return new handle
   */
  virtual CollObjectHandle createHandle() = 0;

  /// Tells all handlers that interactive mode is active/inactive.
  virtual void enableInteraction(bool enable) = 0;

  /// Retrieves the current interaction mode (active/inactive).
  virtual bool getInteractionEnabled() const = 0;

  /**
   * Gives access to all managed handlers in form of a container that can be accessed via a
   * forward iterator.
   * Its usage needs to be protected by explicitly calling lock
   * \return container of all handlers
   */
  virtual HandlerRange handlers() = 0;
};

using HandlerManagerIfacePtr = std::shared_ptr<HandlerManagerIface>;

}  // namespace interaction
}  // namespace rviz_common

#endif  // RVIZ_COMMON__INTERACTION__HANDLER_MANAGER_IFACE_HPP_
