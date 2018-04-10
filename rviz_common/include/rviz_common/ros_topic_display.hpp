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
#ifndef RVIZ_COMMON__ROS_TOPIC_DISPLAY_HPP_
#define RVIZ_COMMON__ROS_TOPIC_DISPLAY_HPP_

#ifndef Q_MOC_RUN

#include <string>

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# pragma GCC diagnostic ignored "-Wpedantic"
#endif
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#endif

#include "rmw/types.h"
#include "rclcpp/rclcpp.hpp"

#include "rviz_common/display.hpp"
#include "rviz_common/display_context.hpp"
#include "frame_manager_iface.hpp"
#include "rviz_common/properties/ros_topic_property.hpp"
#include "rviz_common/properties/status_property.hpp"
#include "rviz_common/visibility_control.hpp"

static const rmw_qos_profile_t display_default =
{
  RMW_QOS_POLICY_HISTORY_KEEP_LAST,
  5,
  RMW_QOS_POLICY_RELIABILITY_RELIABLE,
  RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT,
  false
};

namespace rviz_common
{

/** @brief Helper superclass for RosTopicDisplay, needed because
 * Qt's moc and c++ templates don't work nicely together.  Not
 * intended to be used directly. */
class RVIZ_COMMON_PUBLIC _RosTopicDisplay : public Display
{
  Q_OBJECT

public:
  _RosTopicDisplay()
  : node_(nullptr),
    qos_profile(display_default)
  {
    topic_property_ = new properties::RosTopicProperty("Topic", "",
        "", "", this, SLOT(updateTopic()));
    unreliable_property_ = new properties::BoolProperty(
      "Unreliable", false, "Prefer UDP topic transport", this, SLOT(updateReliability()));
  }

  ~_RosTopicDisplay() override
  {
    if (context_) {
      context_->removeNodeFromMainExecutor(node_);
    }
  }

  using changeQoSProfile = std::function<rmw_qos_profile_t(rmw_qos_profile_t)>;
  virtual void updateQoSProfile(changeQoSProfile change_profile) = 0;

  void onInitialize() override
  {
    // TODO(Martin-Idel-SI): Figure out whether we still need the threaded queue or executor here
    // threaded_nh_.setCallbackQueue(context_->getThreadedQueue());
    node_ = rclcpp::Node::make_shared("display_node");
    context_->addNodeToMainExecutor(node_);
  }

protected Q_SLOTS:
  virtual void updateTopic() = 0;
  virtual void updateReliability() = 0;

protected:
  /** @brief A Node which is registered with the main executor (used in the "update" thread).
   *
   * This is configured after the constructor within the initialize() method of Display. */
  rclcpp::Node::SharedPtr node_;
  rmw_qos_profile_t qos_profile;
  properties::RosTopicProperty * topic_property_;
  properties::BoolProperty * unreliable_property_;
};

/** @brief Display subclass using a rclcpp::subscription, templated on the ROS message type.
 *
 * This class handles subscribing and unsubscribing to a ROS node when the display is
 * enabled or disabled. */
template<class MessageType>
class RosTopicDisplay : public _RosTopicDisplay
{
// No Q_OBJECT macro here, moc does not support Q_OBJECT in a templated class.

public:
  /** @brief Convenience typedef so subclasses don't have to use
   * the long templated class name to refer to their super class. */
  typedef RosTopicDisplay<MessageType> RTDClass;

  RosTopicDisplay()
  : messages_received_(0)
  {
    // TODO(Martin-Idel-SI): We need a way to extract the MessageType from the template to set a
    // correct string. Previously was:
    // QString message_type = QString::fromStdString(ros::message_traits::datatype<MessageType>());
    QString message_type = QString::fromStdString("");
    topic_property_->setMessageType(message_type);
    topic_property_->setDescription(message_type + " topic to subscribe to.");
  }

  ~RosTopicDisplay() override
  {
    unsubscribe();
  }

  /**
   * When overriding this method, the onInitialize() method of this superclass has to be called.
   * Otherwise, the ros node will not be initialized.
   */
  void onInitialize() override
  {
    _RosTopicDisplay::onInitialize();
    topic_property_->initialize(node_);
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

  void updateQoSProfile(changeQoSProfile change_profile) override
  {
    qos_profile = change_profile(qos_profile);
    if (node_) {
      updateTopic();
    }
  }

protected:
  void updateTopic() override
  {
    unsubscribe();
    reset();
    subscribe();
    context_->queueRender();
  }

  void updateReliability() override
  {
    qos_profile.reliability = unreliable_property_->getBool() ?
      RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT :
      RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    updateTopic();
  }

  virtual void subscribe()
  {
    if (!isEnabled() ) {
      return;
    }

    if (topic_property_->isEmpty()) {
      setStatus(properties::StatusProperty::Error,
        "Topic",
        QString("Error subscribing: Empty topic name"));
      return;
    }

    try {
      subscription = node_->create_subscription<MessageType>(
        topic_property_->getTopicStd(),
        std::bind(&RosTopicDisplay<MessageType>::incomingMessage, this, std::placeholders::_1),
        qos_profile);
      setStatus(properties::StatusProperty::Ok, "Topic", "OK");
    } catch (rclcpp::exceptions::InvalidTopicNameError & e) {
      setStatus(properties::StatusProperty::Error, "Topic",
        QString("Error subscribing: ") + e.what());
    }
  }

  virtual void unsubscribe()
  {
    subscription.reset();
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

  /** @brief Implement this to process the contents of a message.
   *
   * This is called by incomingMessage(). */
  virtual void processMessage(typename MessageType::ConstSharedPtr msg) = 0;

  typename rclcpp::Subscription<MessageType>::SharedPtr subscription;
  uint32_t messages_received_;
};

}  // end namespace rviz_common

#endif  // RVIZ_COMMON__ROS_TOPIC_DISPLAY_HPP_
