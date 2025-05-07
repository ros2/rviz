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

#ifndef RVIZ_COMMON__ROS_ADAPTER_DISPLAY_HPP_
#define RVIZ_COMMON__ROS_ADAPTER_DISPLAY_HPP_

#include <cstdint>
#include <functional>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include <QString>  // NOLINT: cpplint is unable to handle the include order here

#include "rviz_common/ros_topic_display.hpp"

#include "rclcpp/qos.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/subscription_options.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/type_adapter.hpp"

namespace rviz_common
{

/** @brief Display subclass using a rclcpp::subscription,
 * templated on the ROS message type and a custom type, to support REP 2007
 *
 * This class handles subscribing and unsubscribing to a ROS node when the display is
 * enabled or disabled. */
template<class MessageType, class AdaptedType>
class RosAdapterDisplay : public rviz_common::_RosTopicDisplay
{
// No Q_OBJECT macro here, moc does not support Q_OBJECT in a templated class.

public:
  /** @brief Convenience typedef so subclasses don't have to use
   * the long templated class name to refer to their super class. */
  typedef RosAdapterDisplay<MessageType, AdaptedType> RADClass;
  using Adapter = typename rclcpp::adapt_type<AdaptedType>::as<MessageType>;

  RosAdapterDisplay()
  : messages_received_(0)
  {
    QString message_type = QString::fromStdString(rosidl_generator_traits::name<MessageType>());
    topic_property_->setMessageType(message_type);
    topic_property_->setDescription(message_type + " topic to subscribe to.");
  }

  ~RosAdapterDisplay() override
  {
    unsubscribe();
  }

  void reset() override
  {
    Display::reset();
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
    unsubscribe();
    reset();
    subscribe();
    context_->queueRender();
  }

  virtual void subscribe()
  {
    if (!isEnabled() ) {
      return;
    }

    if (topic_property_->isEmpty()) {
      setStatus(
        properties::StatusProperty::Error,
        "Topic",
        QString("Error subscribing: Empty topic name"));
      return;
    }

    try {
      rclcpp::SubscriptionOptions sub_opts;
      sub_opts.event_callbacks.message_lost_callback =
        [&](rclcpp::QOSMessageLostInfo & info)
        {
          std::ostringstream sstm;
          sstm << "Some messages were lost:\n>\tNumber of new lost messages: " <<
            info.total_count_change << " \n>\tTotal number of messages lost: " <<
            info.total_count;
          setStatus(properties::StatusProperty::Warn, "Topic", QString(sstm.str().c_str()));
        };

      rclcpp::Node::SharedPtr node = rviz_ros_node_.lock()->get_raw_node();
      subscription_ =
        node->template create_subscription<Adapter>(
        topic_property_->getTopicStd(),
        qos_profile,
        std::bind(
          &RosAdapterDisplay<MessageType, AdaptedType>::incomingMessage, this,
          std::placeholders::_1),
        sub_opts);
      subscription_start_time_ = node->now();
      setStatus(rviz_common::properties::StatusProperty::Ok, "Topic", "OK");
    } catch (rclcpp::exceptions::InvalidTopicNameError & e) {
      setStatus(
        rviz_common::properties::StatusProperty::Error, "Topic",
        QString("Error subscribing: ") + e.what());
    }
  }

  virtual void unsubscribe()
  {
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
    reset();
  }

  /** @brief Incoming message callback.  Checks if the message pointer
   * is valid, increments messages_received_, then calls
   * processMessage(). */
  void incomingMessage(const typename std::shared_ptr<const AdaptedType> msg)
  {
    if (!msg) {
      return;
    }

    ++messages_received_;
    QString topic_str = QString::number(messages_received_) + " messages received";
    rviz_common::properties::StatusProperty::Level topic_status_level =
      rviz_common::properties::StatusProperty::Ok;
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

  /** @brief Implement this to process the contents of a message.
   *
   * This is called by incomingMessage(). */
  virtual void processMessage(const typename std::shared_ptr<const AdaptedType> msg) = 0;

  typename rclcpp::Subscription<Adapter>::SharedPtr subscription_;
  rclcpp::Time subscription_start_time_;
  uint32_t messages_received_;
};

}  // end namespace rviz_common

#endif  // RVIZ_COMMON__ROS_ADAPTER_DISPLAY_HPP_
