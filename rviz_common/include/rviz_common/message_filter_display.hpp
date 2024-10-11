// Copyright (c) 2012, Willow Garage, Inc.
// Copyright (c) 2017, Bosch Software Innovations GmbH.
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

#ifndef RVIZ_COMMON__MESSAGE_FILTER_DISPLAY_HPP_
#define RVIZ_COMMON__MESSAGE_FILTER_DISPLAY_HPP_

#include <tf2_ros/message_filter.h>
#include <memory>
#include <string>

#include <message_filters/subscriber.hpp>

#include "rviz_common/ros_topic_display.hpp"
#include "rviz_common/properties/int_property.hpp"

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
    QString message_type = rosidl_generator_traits::name<MessageType>();
    topic_property_->setMessageType(message_type);
    topic_property_->setDescription(message_type + " topic to subscribe to.");

    message_queue_property_ = new properties::IntProperty(
      "Filter size", 10,
      "Set the filter size of the Message Filter Display.",
      topic_property_, SLOT(updateMessageQueueSize()), this,
      1, INT_MAX);
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
    // Change the QoS according to the info in the ROS Graph
    const auto & publishers = rviz_ros_node_.lock()->get_raw_node()->get_publishers_info_by_topic(
      topic_property_->getTopicStd());

    if (!publishers.empty()) {
      // Check if there is any Subscriber with the same QoS as set
      bool any_publisher_qos_compatible = false;
      for (const auto & publisher : publishers) {
        if (qos_profile == publisher.qos_profile()) {
          any_publisher_qos_compatible = true;
          break;
        }
      }

      // If not found, select one valid
      if (!any_publisher_qos_compatible) {
        qos_profile = publishers[0].qos_profile();
        qos_profile_property_->setQoSProfile(qos_profile);
      }
    }

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

    const auto & publishers = rviz_ros_node_.lock()->get_raw_node()->get_publishers_info_by_topic(
      topic_property_->getTopicStd());

    bool different_qos = false;
    for (const auto & publisher : publishers) {
      if (qos_profile != publisher.qos_profile()) {
        different_qos = true;
        break;
      }
    }

    try {
      rclcpp::Node::SharedPtr node = rviz_ros_node_.lock()->get_raw_node();
      subscription_ = std::make_shared<message_filters::Subscriber<MessageType>>(
        node,
        topic_property_->getTopicStd(),
        qos_profile);
      subscription_start_time_ = node->now();
      tf_filter_ =
        std::make_shared<tf2_ros::MessageFilter<MessageType, transformation::FrameTransformer>>(
        *context_->getFrameManager()->getTransformer(),
        fixed_frame_.toStdString(),
        static_cast<uint32_t>(message_queue_property_->getInt()),
        node);
      tf_filter_->connectInput(*subscription_);
      tf_filter_->registerCallback(
        std::bind(
          &MessageFilterDisplay<MessageType>::messageTaken, this,
          std::placeholders::_1));

      if (!different_qos) {
        setStatus(properties::StatusProperty::Ok, "Topic", "OK");
      } else {
        setStatus(
          properties::StatusProperty::Warn, "Topic",
          "1 or more publishers have different QoS settings");
      }
    } catch (rclcpp::exceptions::InvalidTopicNameError & e) {
      setStatus(
        properties::StatusProperty::Error, "Topic", QString("Error subscribing: ") + e.what());
    }
  }

  void updateMessageQueueSize()
  {
    if (tf_filter_) {
      tf_filter_->setQueueSize(static_cast<uint32_t>(message_queue_property_->getInt()));
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
    tf_filter_.reset();
    subscription_.reset();
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

  void messageTaken(typename MessageType::ConstSharedPtr msg)
  {
    if (!msg) {
      return;
    }

    // Do not process message right away, tf2_ros::MessageFilter may be
    // calling back from tf2_ros::TransformListener dedicated thread.
    // Use type erased signal/slot machinery to ensure messages are
    // processed in the main thread.
    Q_EMIT typeErasedMessageTaken(std::static_pointer_cast<const void>(msg));
  }

  void processTypeErasedMessage(std::shared_ptr<const void> type_erased_msg) override
  {
    auto msg = std::static_pointer_cast<const MessageType>(type_erased_msg);

    ++messages_received_;
    rviz_common::properties::StatusProperty::Level topic_status_level =
      rviz_common::properties::StatusProperty::Ok;
    QString topic_str = QString::number(messages_received_) + " messages received";
    // Append topic subscription frequency if we can lock rviz_ros_node_.
    std::shared_ptr<ros_integration::RosNodeAbstractionIface> node_interface =
      rviz_ros_node_.lock();
    if (node_interface != nullptr) {
      try {
        const double duration =
          (node_interface->get_raw_node()->now() - subscription_start_time_).seconds();
        const double subscription_frequency =
          static_cast<double>(messages_received_) / duration;
        topic_str += " at " + QString::number(subscription_frequency, 'f', 1) + " hz.";
      } catch (const std::runtime_error & e) {
        if (std::string(e.what()).find("can't subtract times with different time sources") !=
          std::string::npos)
        {
          topic_status_level = rviz_common::properties::StatusProperty::Warn;
          topic_str += ". ";
          topic_str += e.what();
        } else {
          throw;
        }
      }
    }
    setStatus(
      topic_status_level,
      "Topic",
      topic_str);

    processMessage(msg);
  }

  /// Implement this to process the contents of a message.
  /**
   * This is called by incomingMessage().
   */
  virtual void processMessage(typename MessageType::ConstSharedPtr msg) = 0;

  typename std::shared_ptr<message_filters::Subscriber<MessageType>> subscription_;
  rclcpp::Time subscription_start_time_;
  std::shared_ptr<tf2_ros::MessageFilter<MessageType, transformation::FrameTransformer>> tf_filter_;
  uint32_t messages_received_;
  properties::IntProperty * message_queue_property_;
};

}  // end namespace rviz_common

#endif  // RVIZ_COMMON__MESSAGE_FILTER_DISPLAY_HPP_
