/*
 * Copyright (c) 2012, Willow Garage, Inc.
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
#ifndef RVIZ_COMMON__MESSAGE_FILTER_DISPLAY_HPP_
#define RVIZ_COMMON__MESSAGE_FILTER_DISPLAY_HPP_

#include <message_filters/subscriber.h>
#include <tf2_ros/message_filter.h>

#include <memory>

#include "rviz_common/ros_topic_display.hpp"

namespace rviz_common
{
/// Display subclass using a rclcpp::subscription and tf2_ros::MessageFilter.
/**
 * This class handles subscribing and unsubscribing to a ROS node using the
 * message filter when the display is enabled or disabled.
 * This class is templated on the ROS message type.
 */
template<class MessageType>
class MessageFilterDisplay : public _RosTopicDisplay
{
// No Q_OBJECT macro here, moc does not support Q_OBJECT in a templated class.

public:
  /// Convenience typedef so subclasses don't have to use
  /// the long templated class name to refer to their super class.
  typedef MessageFilterDisplay<MessageType> MFDClass;

  MessageFilterDisplay()
  : tf_filter_(nullptr),
    messages_received_(0)
  {
    // TODO(Martin-Idel-SI): We need a way to extract the MessageType from the template to set a
    // correct string. Previously was:
    // QString message_type =
    //   QString::fromStdString(message_filters::message_traits::datatype<MessageType>());
    QString message_type = QString::fromStdString("");
    topic_property_->setMessageType(message_type);
    topic_property_->setDescription(message_type + " topic to subscribe to.");
  }

  /**
   * When overriding this method, the onInitialize() method of this superclass has to be called.
   * Otherwise, the ros node will not be initialized.
   */
  void onInitialize() override
  {
    _RosTopicDisplay::onInitialize();
  }

  ~MessageFilterDisplay() override
  {
    unsubscribe();
  }

  void reset() override
  {
    Display::reset();
    if (tf_filter_) {
      tf_filter_->clear();
    }
    messages_received_ = 0;
  }

  void setTopic(const QString & topic, const QString & datatype) override
  {
    (void) datatype;
    topic_property_->setString(topic);
  }

protected:
  void updateTopic() override
  {
    resetSubscription();
  }

  virtual void subscribe()
  {
    if (!isEnabled()) {
      return;
    }

    if (topic_property_->isEmpty()) {
      setStatus(
        properties::StatusProperty::Error, "Topic", QString("Error subscribing: Empty topic name"));
      return;
    }

    try {
      subscription_ = std::make_shared<message_filters::Subscriber<MessageType>>(
        rviz_ros_node_.lock()->get_raw_node(),
        topic_property_->getTopicStd(),
        qos_profile.get_rmw_qos_profile());
      tf_filter_ =
        std::make_shared<tf2_ros::MessageFilter<MessageType, transformation::FrameTransformer>>(
        *context_->getFrameManager()->getTransformer(),
        fixed_frame_.toStdString(), 10, rviz_ros_node_.lock()->get_raw_node());
      tf_filter_->connectInput(*subscription_);
      tf_filter_->registerCallback(
        std::bind(
          &MessageFilterDisplay<MessageType>::incomingMessage, this,
          std::placeholders::_1));
      setStatus(properties::StatusProperty::Ok, "Topic", "OK");
    } catch (rclcpp::exceptions::InvalidTopicNameError & e) {
      setStatus(
        properties::StatusProperty::Error, "Topic", QString("Error subscribing: ") + e.what());
    }
  }

  void transformerChangedCallback() override
  {
    resetSubscription();
  }

  void resetSubscription()
  {
    unsubscribe();
    reset();
    subscribe();
    context_->queueRender();
  }

  virtual void unsubscribe()
  {
    subscription_.reset();
    tf_filter_.reset();
  }

  void onEnable() override
  {
    subscribe();
  }

  void onDisable() override
  {
    unsubscribe();
    reset();
  }

  void fixedFrameChanged() override
  {
    if (tf_filter_) {
      tf_filter_->setTargetFrame(fixed_frame_.toStdString());
    }
    reset();
  }

  /// Incoming message callback.
  /**
   * Checks if the message pointer
   * is valid, increments messages_received_, then calls
   * processMessage().
   */
  void incomingMessage(const typename MessageType::ConstSharedPtr msg)
  {
    if (!msg) {
      return;
    }

    ++messages_received_;
    setStatus(
      properties::StatusProperty::Ok,
      "Topic",
      QString::number(messages_received_) + " messages received");

    processMessage(msg);
  }

  /// Implement this to process the contents of a message.
  /**
   * This is called by incomingMessage().
   */
  virtual void processMessage(typename MessageType::ConstSharedPtr msg) = 0;

  typename std::shared_ptr<message_filters::Subscriber<MessageType>> subscription_;
  std::shared_ptr<tf2_ros::MessageFilter<MessageType, transformation::FrameTransformer>> tf_filter_;
  uint32_t messages_received_;
};

}  // end namespace rviz_common

#endif  // RVIZ_COMMON__MESSAGE_FILTER_DISPLAY_HPP_
