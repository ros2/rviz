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

#ifndef RVIZ_COMMON__PROPERTIES__QUEUE_SIZE_PROPERTY_HPP_
#define RVIZ_COMMON__PROPERTIES__QUEUE_SIZE_PROPERTY_HPP_

#include "rmw/types.h"

#include "rviz_common/ros_topic_display.hpp"
#include "rviz_common/properties/int_property.hpp"
#include "rviz_common/visibility_control.hpp"

namespace rviz_common
{

class RVIZ_COMMON_PUBLIC QueueSizeProperty : public QObject
{
  Q_OBJECT

public:
  QueueSizeProperty(_RosTopicDisplay * display, uint32_t default_size);

  void setDescription(const QString & description);

private Q_SLOTS:
  void updateQueueSize();

private:
  rviz_common::properties::IntProperty * queue_size_property_;
  rviz_common::_RosTopicDisplay * display_;
};
}  // namespace rviz_common

#endif  // RVIZ_COMMON__PROPERTIES__QUEUE_SIZE_PROPERTY_HPP_
