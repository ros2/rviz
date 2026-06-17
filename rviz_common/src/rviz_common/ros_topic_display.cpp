// Copyright (c) 2026, Bosch Software Innovations GmbH.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "rviz_common/ros_topic_display.hpp"

#include <memory>

namespace rviz_common
{

_RosTopicDisplay::_RosTopicDisplay()
: rviz_ros_node_(),
  qos_profile(5)
{
  qRegisterMetaType<std::shared_ptr<const void>>();

  topic_property_ = new properties::RosTopicProperty(
    "Topic", "",
    "", "", this, SLOT(updateTopic()));

  qos_profile_property_ = new properties::QosProfileProperty(topic_property_, qos_profile);
}

void _RosTopicDisplay::onInitialize()
{
  rviz_ros_node_ = context_->getRosNodeAbstraction();
  topic_property_->initialize(rviz_ros_node_);

  connect(
    reinterpret_cast<QObject *>(context_->getTransformationManager()),
    SIGNAL(transformerChanged(std::shared_ptr<rviz_common::transformation::FrameTransformer>)),
    this,
    SLOT(transformerChangedCallback()));
  qos_profile_property_->initialize(
    [this](rclcpp::QoS profile) {
      this->qos_profile = profile;
      updateTopic();
    });

  // Useful to _ROSTopicDisplay subclasses to ensure GUI updates
  // are performed by the main thread only.
  connect(
    this,
    SIGNAL(typeErasedMessageTaken(std::shared_ptr<const void>)),
    this,
    SLOT(processTypeErasedMessage(std::shared_ptr<const void>)),
    // Force queued connections regardless of QObject thread affinity
    Qt::QueuedConnection);
}

void _RosTopicDisplay::processTypeErasedMessage(std::shared_ptr<const void> type_erased_message)
{
  (void)type_erased_message;
}

void _RosTopicDisplay::transformerChangedCallback()
{
}

void _RosTopicDisplay::updateMessageQueueSize()
{
}

}  // namespace rviz_common
