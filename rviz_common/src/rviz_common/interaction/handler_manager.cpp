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

#include "rviz_common/interaction/handler_manager.hpp"

#include <mutex>
#include <utility>


namespace rviz_common
{
namespace interaction
{

HandlerManager::HandlerManager()
: uid_counter_(0),
  interaction_enabled_(false)
{}

HandlerManager::~HandlerManager()
{
  std::lock_guard<std::recursive_mutex> lock(handlers_mutex_);
  handlers_.clear();
}

void HandlerManager::addHandler(CollObjectHandle handle, SelectionHandlerWeakPtr handler)
{
  if (!handle) {
    return;
  }

  std::lock_guard<std::recursive_mutex> lock(handlers_mutex_);

  auto handler_shared_ptr = handler.lock();
  InteractiveObjectPtr object = handler_shared_ptr->getInteractiveObject().lock();
  if (object) {
    object->enableInteraction(interaction_enabled_);
  }

  bool inserted = handlers_.insert(std::make_pair(handle, handler)).second;
  (void) inserted;
  assert(inserted);
}

void HandlerManager::removeHandler(CollObjectHandle handle)
{
  if (!handle) {
    return;
  }

  std::lock_guard<std::recursive_mutex> lock(handlers_mutex_);

  handlers_.erase(handle);

  for (const auto & listener : listeners_) {
    listener->onHandlerRemoved(handle);
  }
}

void HandlerManager::addListener(HandlerManagerListener * listener)
{
  listeners_.emplace_back(listener);
}

void HandlerManager::removeListener(HandlerManagerListener * listener)
{
  listeners_.erase(std::remove(listeners_.begin(), listeners_.end(), listener), listeners_.end());
}

SelectionHandlerPtr HandlerManager::getHandler(CollObjectHandle handle)
{
  std::lock_guard<std::recursive_mutex> lock(handlers_mutex_);

  auto item = handlers_.find(handle);
  return item != handlers_.end() ? item->second.lock() : nullptr;
}

std::unique_lock<std::recursive_mutex> HandlerManager::lock()
{
  return std::unique_lock<std::recursive_mutex>(handlers_mutex_);
}

std::unique_lock<std::recursive_mutex> HandlerManager::lock(std::defer_lock_t defer_lock)
{
  return std::unique_lock<std::recursive_mutex>(handlers_mutex_, defer_lock);
}

CollObjectHandle HandlerManager::createHandle()
{
  std::lock_guard<std::recursive_mutex> lock(uid_mutex_);

  uid_counter_++;
  if (uid_counter_ > 0x00ffffff) {
    uid_counter_ = 0;
  }

  CollObjectHandle handle = 0;

  // shuffle around the bits so we get lots of colors
  // when we're displaying the selection buffer
  for (unsigned int i = 0; i < 24; i++) {
    uint32_t shift = (((23 - i) % 3) * 8) + (23 - i) / 3;
    uint32_t bit = ( (uint32_t)(uid_counter_ >> i) & (uint32_t)1) << shift;
    handle |= bit;
  }

  return handle;
}

void HandlerManager::enableInteraction(bool enable)
{
  interaction_enabled_ = enable;
  std::lock_guard<std::recursive_mutex> lock(handlers_mutex_);
  for (auto handler : handlers_) {
    if (InteractiveObjectPtr object = handler.second.lock()->getInteractiveObject().lock()) {
      object->enableInteraction(enable);
    }
  }
}

bool HandlerManager::getInteractionEnabled() const
{
  return interaction_enabled_;
}

HandlerRange HandlerManager::handlers()
{
  return HandlerRange(handlers_);
}

}  // namespace interaction
}  // namespace rviz_common
