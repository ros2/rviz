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

#include "rviz_common/properties/queue_size_property.hpp"

namespace rviz_common
{

QueueSizeProperty::QueueSizeProperty(_RosTopicDisplay * display, uint32_t default_size)
: display_(display)
{
  queue_size_property_ = new rviz_common::properties::IntProperty(
    "Queue Size", default_size,
    "Advanced: set the size of the incoming message queue. Increasing this is useful if your "
    "incoming TF data is delayed significantly from your message data, but it can greatly "
    "increase memory usage if the messages are big.",
    display_, SLOT(updateQueueSize()), this);

  updateQueueSize();
}

void QueueSizeProperty::setDescription(const QString & description)
{
  queue_size_property_->setDescription(description);
}

void QueueSizeProperty::updateQueueSize()
{
  display_->updateQoSProfile([this](rmw_qos_profile_t profile) -> rmw_qos_profile_t {
      profile.depth = this->queue_size_property_->getInt();
      return profile;
    });
}

}  // namespace rviz_common
