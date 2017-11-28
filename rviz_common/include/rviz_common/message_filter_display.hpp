/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

<<<<<<< 4c9290fe3ff7aadc3822f7e661ebbc75153e9e41
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
#endif
=======
#ifndef Q_MOC_RUN
>>>>>>> Reintroduce RosTopicProperty to MessageFilterDisplays

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// TODO(greimela): Add again as soon as the message filter subscriber is ported to ROS 2
// #include <message_filters/subscriber.h>
// #include <tf/message_filter.h>

#include "rviz_common/display_context.hpp"
#include "rviz_common/frame_manager.hpp"
#include "rviz_common/properties/status_property.hpp"
#include "rviz_common/properties/ros_topic_property.hpp"

#include "rviz_common/display.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace rviz_common
{

/** @brief Helper superclass for MessageFilterDisplay, needed because
 * Qt's moc and c++ templates don't work nicely together.  Not
 * intended to be used directly. */
class _RosTopicDisplay : public Display
{
Q_OBJECT

public:
  _RosTopicDisplay()
  {
    topic_property_ = new properties::RosTopicProperty("Topic", "",
      "", "", this, SLOT(updateTopic()));
    unreliable_property_ = new properties::BoolProperty(
      "Unreliable", false, "Prefer UDP topic transport", this, SLOT(updateTopic()));
  }

protected Q_SLOTS:

  virtual void updateTopic() = 0;

protected:
  properties::RosTopicProperty * topic_property_;
  properties::BoolProperty * unreliable_property_;
};

/** @brief Display subclass using a tf::MessageFilter, templated on the ROS message type.
 *
 * This class brings together some common things used in many Display
 * types.  It has a tf::MessageFilter to filter incoming messages, and
 * it handles subscribing and unsubscribing when the display is
 * enabled or disabled.  It also has an Ogre::SceneNode which  */
template<class MessageType>
class MessageFilterDisplay : public _RosTopicDisplay
{
// No Q_OBJECT macro here, moc does not support Q_OBJECT in a templated class.

public:
  /** @brief Convenience typedef so subclasses don't have to use
   * the long templated class name to refer to their super class. */
  typedef MessageFilterDisplay<MessageType> MFDClass;

  MessageFilterDisplay()
//    : tf_filter_( NULL )
    : messages_received_(0)
  {
    // TODO(Martin-Idel-SI): The message type should be the type of <MessageType>
    //    Currently, we can't get the trait. Was:
    // QString message_type = QString::fromStdString(ros::message_traits::datatype<MessageType>());
    QString message_type = QString::fromStdString("");
    topic_property_->setMessageType(message_type);
    topic_property_->setDescription(message_type + " topic to subscribe to.");
  }

  virtual void onInitialize()
  {
    topic_property_->initialise(node_);
    // TODO(Martin-Idel-SI): revisit once MessageFilter is ported
//      tf_filter_ = new tf::MessageFilter<MessageType>( *context_->getTFClient(),
//                                                fixed_frame_.toStdString(), 10, update_nh_ );
//
//      tf_filter_->connectInput( sub_ );
//      tf_filter_->registerCallback( boost::bind(
//           &MessageFilterDisplay<MessageType>::incomingMessage, this, _1 ));
//      context_->getFrameManager()->registerFilterForTransformStatusCheck( tf_filter_, this );
    topic_property_->initialize(node_);
  }

  virtual ~MessageFilterDisplay()
  {
    unsubscribe();
//      delete tf_filter_;
  }

  virtual void reset()
  {
    Display::reset();
//      tf_filter_->clear();
    messages_received_ = 0;
  }

  virtual void setTopic(const QString & topic, const QString & datatype)
  {
    (void) datatype;
    topic_property_->setString(topic);
  }

protected:
  virtual void updateTopic()
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
      setStatus(properties::StatusProperty::Error,
        "Topic",
        QString("Error subscribing: Empty topic name"));
      return;
    }

    try {
      // TODO(greimela): Revisit after MessageFilters are migrated
//        ros::TransportHints transport_hint = ros::TransportHints().reliable();
//        // Determine UDP vs TCP transport for user selection.
//        if (unreliable_property_->getBool())
//        {
//          transport_hint = ros::TransportHints().unreliable();
//        }
      subscription = node_->create_subscription<MessageType>(
        topic_property_->getTopicStd(),
        std::bind(&MessageFilterDisplay<MessageType>::incomingMessage, this,
          std::placeholders::_1));
      setStatus(properties::StatusProperty::Ok, "Topic", "OK");
    }
    catch (rclcpp::exceptions::InvalidTopicNameError & e) {
      setStatus(properties::StatusProperty::Error, "Topic",
        QString("Error subscribing: ") + e.what());
    }
  }

  virtual void unsubscribe()
  {
    subscription.reset();
  }

  virtual void onEnable()
  {
    subscribe();
  }

  virtual void onDisable()
  {
    unsubscribe();
    reset();
  }

  virtual void fixedFrameChanged()
  {
    // TODO(greimela): Revisit after MessageFilters are migrated
//      tf_filter_->setTargetFrame( fixed_frame_.toStdString() );
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

  // TODO(greimela): Revisit after MessageFilters are migrated
  //  message_filters::Subscriber<MessageType> sub_;
//  tf::MessageFilter<MessageType>* tf_filter_;
  typename rclcpp::Subscription<MessageType>::SharedPtr subscription;
  uint32_t messages_received_;
};

}  // end namespace rviz_common

#endif  // RVIZ_COMMON__MESSAGE_FILTER_DISPLAY_HPP_
