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

#include <mutex>
#include <unordered_map>

#include "rviz_common/interaction/handler_manager_listener.hpp"
#include "rviz_common/visibility_control.hpp"

#include "./selection_handler.hpp"


namespace rviz_common
{

class DisplayContext;

namespace interaction
{

using M_ObjectHandleToSelectionHandler =
  std::unordered_map<CollObjectHandle, SelectionHandlerWeakPtr>;

class RVIZ_COMMON_PUBLIC HandlerManagerIface
{
public:
  virtual ~HandlerManagerIface() = default;

  virtual void addHandler(CollObjectHandle handle, SelectionHandlerWeakPtr handler) = 0;

  virtual void removeHandler(CollObjectHandle handle) = 0;

  virtual SelectionHandlerPtr getHandler(CollObjectHandle handle) = 0;

  virtual std::unique_lock<std::recursive_mutex> lock() = 0;

  virtual void addListener(HandlerManagerListener * listener) = 0;

  virtual void removeListener(HandlerManagerListener * listener) = 0;

  /// Create a new unique handle.
  virtual CollObjectHandle createHandle() = 0;

  /// Tell all handlers that interactive mode is active/inactive.
  virtual void enableInteraction(bool enable) = 0;

  virtual bool getInteractionEnabled() const = 0;

  M_ObjectHandleToSelectionHandler handlers_;
};

}  // namespace interaction
}  // namespace rviz_common

#endif  // RVIZ_COMMON__INTERACTION__HANDLER_MANAGER_IFACE_HPP_
