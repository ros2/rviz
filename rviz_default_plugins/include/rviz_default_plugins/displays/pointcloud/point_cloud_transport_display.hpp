// Copyright (c) 2023, Open Source Robotics Foundation, Inc.
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

#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__POINTCLOUD__POINT_CLOUD_TRANSPORT_DISPLAY_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__POINTCLOUD__POINT_CLOUD_TRANSPORT_DISPLAY_HPP_

#include <memory>
#include <string>

#include <QString>  // NOLINT: cpplint is unable to handle the include order here

#include "get_transport_from_topic.hpp"
#include "point_cloud_transport/point_cloud_transport.hpp"
#include "point_cloud_transport/subscriber_filter.hpp"
#include "rviz_common/message_filter_display.hpp"

namespace rviz_default_plugins
{
namespace displays
{

template<class MessageType>
class PointCloud2TransportDisplay : public rviz_common::MessageFilterDisplay<MessageType>
{
  // No Q_OBJECT macro here, moc does not support Q_OBJECT in a templated class.

public:
  /// Convenience typedef so subclasses don't have to use
  /// the long templated class name to refer to their super class.
  typedef PointCloud2TransportDisplay<MessageType> PC2RDClass;

protected:
  void subscribe() override
  {
    if (!this->isEnabled()) {
      return;
    }

    if (this->topic_property_->isEmpty()) {
      this->setStatus(rviz_common::properties::StatusProperty::Error, "Topic",
                      QString("Error subscribing: Empty topic name"));
      return;
    }

    try {
      rclcpp::Node::SharedPtr node = this->rviz_ros_node_.lock()->get_raw_node();
      subscriber_filter_ = std::make_shared<point_cloud_transport::SubscriberFilter>();
      subscriber_filter_->subscribe(
        rclcpp::node_interfaces::NodeInterfaces<
          rclcpp::node_interfaces::NodeBaseInterface,
          rclcpp::node_interfaces::NodeParametersInterface,
          rclcpp::node_interfaces::NodeTopicsInterface,
          rclcpp::node_interfaces::NodeLoggingInterface>(*node),
        getPointCloud2BaseTopicFromTopic(this->topic_property_->getTopicStd()),
        getPointCloud2TransportFromTopic(this->topic_property_->getTopicStd()),
        this->qos_profile);
      this->subscription_start_time_ = node->now();
      this->tf_filter_ =
        std::make_shared<tf2_ros::MessageFilter<MessageType,
          rviz_common::transformation::FrameTransformer>>(
              *this->context_->getFrameManager()->getTransformer(),
            this->fixed_frame_.toStdString(),
              static_cast<uint32_t>(this->message_queue_property_->getInt()), *node);
      this->tf_filter_->connectInput(*subscriber_filter_);
      this->tf_filter_->registerCallback(
          std::bind(&PointCloud2TransportDisplay<MessageType>::messageTaken, this,
            std::placeholders::_1));
      this->setStatus(rviz_common::properties::StatusProperty::Ok, "Topic", "OK");
    } catch (rclcpp::exceptions::InvalidTopicNameError & e) {
      this->setStatus(rviz_common::properties::StatusProperty::Error, "Topic",
                      QString("Error subscribing: ") + e.what());
    }
  }

  virtual void unsubscribe()
  {
    rviz_common::MessageFilterDisplay<MessageType>::unsubscribe();
    subscriber_filter_.reset();
  }

  std::shared_ptr<point_cloud_transport::SubscriberFilter> subscriber_filter_;
};

}  //  end namespace displays
}  // end namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__POINTCLOUD__POINT_CLOUD_TRANSPORT_DISPLAY_HPP_
