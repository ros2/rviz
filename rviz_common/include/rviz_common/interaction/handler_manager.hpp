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

#ifndef RVIZ_COMMON__INTERACTION__HANDLER_MANAGER_HPP_
#define RVIZ_COMMON__INTERACTION__HANDLER_MANAGER_HPP_

#include "handler_manager_iface.hpp"

#include <mutex>
#include <vector>

#include "rviz_common/interaction/forwards.hpp"
#include "rviz_common/interaction/handler_manager_listener.hpp"
#include "rviz_common/interaction/selection_handler.hpp"
#include "rviz_common/visibility_control.hpp"


namespace rviz_common
{

class DisplayContext;

namespace interaction
{

class RVIZ_COMMON_PUBLIC HandlerManager
  : public HandlerManagerIface
{
public:
  HandlerManager();

  ~HandlerManager() override;

  void addHandler(CollObjectHandle handle, SelectionHandlerWeakPtr handler) override;

  void removeHandler(CollObjectHandle handle) override;

  SelectionHandlerPtr getHandler(CollObjectHandle handle) override;

  std::unique_lock<std::recursive_mutex> lock() override;

  std::unique_lock<std::recursive_mutex> lock(std::defer_lock_t defer_lock) override;

  void addListener(HandlerManagerListener * listener) override;

  void removeListener(HandlerManagerListener * listener) override;

  CollObjectHandle createHandle() override;

  void enableInteraction(bool enable) override;

  bool getInteractionEnabled() const override;

  HandlerRange handlers() override;

private:
  uint32_t uid_counter_;

  bool interaction_enabled_;

  std::recursive_mutex handlers_mutex_;
  std::recursive_mutex uid_mutex_;

  M_ObjectHandleToSelectionHandler handlers_;
  std::vector<HandlerManagerListener *> listeners_;
};

}  // namespace interaction
}  // namespace rviz_common

#endif  // RVIZ_COMMON__INTERACTION__HANDLER_MANAGER_HPP_
