/*
* Copyright (c) 2012, Willow Garage, Inc.
* Copyright (c) 2017, Bosch Software Innovations GmbH.
* Copyright (c) 2020, TNG Technology Consulting GmbH.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted (subject to the limitations in the disclaimer
* below) provided that the following conditions are met:
*
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*     * Neither the name of the copyright holder nor the names of its
*       contributors may be used to endorse or promote products derived from
*       this software without specific prior written permission.
*
* NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
* LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
* THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__IMAGE__IMAGE_TRANSPORT_DISPLAY_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__IMAGE__IMAGE_TRANSPORT_DISPLAY_HPP_

#include <memory>

#include "get_transport_from_topic.hpp"
#include <image_transport/image_transport.hpp>
#include <image_transport/camera_common.hpp>
#include <image_transport/publisher_plugin.hpp>
#include <image_transport/subscriber_plugin.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <pluginlib/class_loader.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/ros_topic_display.hpp>

#include <sensor_msgs/msg/compressed_image.hpp>

namespace rviz_default_plugins
{
namespace displays
{

enum PluginStatus {SUCCESS, CREATE_FAILURE, LIB_LOAD_FAILURE, DOES_NOT_EXIST};

/// \cond
struct TransportDesc
{
  TransportDesc()
  : pub_status(DOES_NOT_EXIST), sub_status(DOES_NOT_EXIST)
  {}

  std::string package_name;
  std::string pub_name;
  PluginStatus pub_status;
  std::string sub_name;
  PluginStatus sub_status;
};

template<class MessageType>
class ImageTransportDisplay : public rviz_common::_RosTopicDisplay
{
// No Q_OBJECT macro here, moc does not support Q_OBJECT in a templated class.

public:
/// Convenience typedef so subclasses don't have to use
/// the long templated class name to refer to their super class.
  typedef ImageTransportDisplay<MessageType> ITDClass;

  ImageTransportDisplay()
  : messages_received_(0)
  {
    QString message_type = QString::fromStdString(rosidl_generator_traits::name<MessageType>());
    topic_property_->setMessageType(message_type);
    topic_property_->setDescription(message_type + " topic to subscribe to.");

    image_transport_type_property_ = new rviz_common::properties::EnumProperty(
      "Transport hints", "raw", "Preferred method of sending images.",
      this, SLOT(updateTopic()));

    pluginlib::ClassLoader<image_transport::PublisherPlugin> pub_loader(
      "image_transport", "image_transport::PublisherPlugin");
    pluginlib::ClassLoader<image_transport::SubscriberPlugin> sub_loader(
      "image_transport", "image_transport::SubscriberPlugin");
    typedef std::map<std::string, TransportDesc> StatusMap;
    StatusMap transports;

    for (const std::string & lookup_name : pub_loader.getDeclaredClasses()) {
      std::string transport_name = image_transport::erase_last_copy(lookup_name, "_pub");
      transports[transport_name].pub_name = lookup_name;
      transports[transport_name].package_name = pub_loader.getClassPackage(lookup_name);
      try {
        auto pub = pub_loader.createUniqueInstance(lookup_name);
        transports[transport_name].pub_status = SUCCESS;
      } catch (const pluginlib::LibraryLoadException &) {
        transports[transport_name].pub_status = LIB_LOAD_FAILURE;
      } catch (const pluginlib::CreateClassException &) {
        transports[transport_name].pub_status = CREATE_FAILURE;
      }
    }

    for (const std::string & lookup_name : sub_loader.getDeclaredClasses()) {
      std::string transport_name = image_transport::erase_last_copy(lookup_name, "_sub");
      transports[transport_name].sub_name = lookup_name;
      transports[transport_name].package_name = sub_loader.getClassPackage(lookup_name);
      try {
        auto sub = sub_loader.createUniqueInstance(lookup_name);
        transports[transport_name].sub_status = SUCCESS;
      } catch (const pluginlib::LibraryLoadException &) {
        transports[transport_name].sub_status = LIB_LOAD_FAILURE;
      } catch (const pluginlib::CreateClassException &) {
        transports[transport_name].sub_status = CREATE_FAILURE;
      }
    }
    for (const StatusMap::value_type & value : transports) {
      std::vector<std::string> tokens = split(
        value.first, '/');
      image_transport_type_property_->addOption(tokens[1].c_str());
    }
  }

/**
* When overriding this method, the onInitialize() method of this superclass has to be called.
* Otherwise, the ros node will not be initialized.
*/
  void onInitialize() override
  {
    _RosTopicDisplay::onInitialize();
  }

  ~ImageTransportDisplay() override
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

  std::vector<std::string> split(const std::string& target, char c)
  {
  	std::string temp;
  	std::stringstream stringstream { target };
  	std::vector<std::string> result;

  	while (std::getline(stringstream, temp, c)) {
  		result.push_back(temp);
  	}

  	return result;
  }


protected:
  void updateTopic() override
  {
    if (image_transport_type_property_->getStdString() == "raw")
    {
      QString message_type =
        QString::fromStdString(rosidl_generator_traits::name<sensor_msgs::msg::Image>());
      topic_property_->setMessageType(message_type);
    } else if (image_transport_type_property_->getStdString() == "compressed")
    {
      QString message_type =
        QString::fromStdString(rosidl_generator_traits::name<sensor_msgs::msg::CompressedImage>());
      topic_property_->setMessageType(message_type);
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
        rviz_common::properties::StatusProperty::Error, "Topic",
        QString("Error subscribing: Empty topic name"));
      return;
    }

    try {
      subscription_ = std::make_shared<image_transport::SubscriberFilter>();
      subscription_->subscribe(
        rviz_ros_node_.lock()->get_raw_node().get(),
        getBaseTopicFromTopic(topic_property_->getTopicStd()),
        getTransportFromTopic(topic_property_->getTopicStd()),
        qos_profile.get_rmw_qos_profile());
      subscription_callback_ = subscription_->registerCallback(
        std::bind(
          &ImageTransportDisplay<MessageType>::incomingMessage, this, std::placeholders::_1));
      setStatus(rviz_common::properties::StatusProperty::Ok, "Topic", "OK");
    } catch (rclcpp::exceptions::InvalidTopicNameError & e) {
      setStatus(
        rviz_common::properties::StatusProperty::Error, "Topic",
        QString("Error subscribing: ") + e.what());
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
      rviz_common::properties::StatusProperty::Ok,
      "Topic",
      QString::number(messages_received_) + " messages received");

    processMessage(msg);
  }

/// Implement this to process the contents of a message.
/**
* This is called by incomingMessage().
*/
  virtual void processMessage(typename MessageType::ConstSharedPtr msg) = 0;

  uint32_t messages_received_;

  std::shared_ptr<image_transport::SubscriberFilter> subscription_;
  message_filters::Connection subscription_callback_;

  rviz_common::properties::EnumProperty * image_transport_type_property_;

private Q_SLOTS:
  void updateChoice()
  {
    std::cerr << "updateChoice" << '\n';
  }

};

}  //  end namespace displays
}  // end namespace rviz_default_plugins


#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__IMAGE__IMAGE_TRANSPORT_DISPLAY_HPP_
