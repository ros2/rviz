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

#include "rviz_default_plugins/displays/image/image_display.hpp"

#include <OgreCamera.h>
#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreRectangle2D.h>
#include <OgreRenderSystem.h>
#include <OgreRenderWindow.h>
#include <OgreRoot.h>
#include <OgreSceneNode.h>
#include <OgreTechnique.h>
#include <OgreTextureManager.h>
#include <OgreViewport.h>
#include <QSet>
#include <QString>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <unordered_set>

#include "image_transport/camera_common.hpp"
#include "image_transport/exception.hpp"
#include "image_transport/image_transport.hpp"
#include "image_transport/subscriber.hpp"
#include "image_transport/subscriber_plugin.hpp"
#include "pluginlib/class_loader.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/frame_manager_iface.hpp"
#include "rviz_common/properties/ros_topic_multi_type_property.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/uniform_string_stream.hpp"
#include "rviz_common/validate_floats.hpp"
#include "rviz_default_plugins/displays/image/get_transport_from_topic.hpp"
#include "rviz_default_plugins/displays/image/ros_image_texture.hpp"
#include "rviz_default_plugins/displays/image/ros_image_texture_iface.hpp"
#include "rviz_rendering/material_manager.hpp"
#include "rviz_rendering/render_window.hpp"
#include "sensor_msgs/image_encodings.hpp"
namespace rviz_default_plugins
{
namespace displays
{

ImageDisplay::ImageDisplay()
: ImageDisplay(std::make_unique<ROSImageTexture>()) {}

ImageDisplay::ImageDisplay(std::unique_ptr<ROSImageTextureIface> texture)
: messages_received_(0),
  texture_(std::move(texture))
{
  // Remove the default single-type topic and replace with a multi-type topic property
  // This allows us to display image and compressed image topics in the topic list
  delete this->topic_property_;
  this->topic_property_ = new rviz_common::properties::RosTopicMultiTypeProperty(
    "Topic", "", QSet<QString>(), "Image transport topic to subscribe to.", this,
    SLOT(updateTopic()));

  delete this->qos_profile_property_;
  this->qos_profile_property_ =
    new rviz_common::properties::QosProfileProperty(this->topic_property_, rclcpp::QoS(5));

  transport_override_property_ = new rviz_common::properties::EnumProperty(
    "Transport Override", "", QString("By default this display uses the topic name to ") +
    QString("determine the image_transport type. If this is not possible, use this ") +
    QString("field to manually set the transport."),
    this->topic_property_, SLOT(subscribe()), this
  );

  normalize_property_ = new rviz_common::properties::BoolProperty(
    "Normalize Range", true,
    "If set to true, will try to estimate the range of possible values from the received images.",
    this, SLOT(updateNormalizeOptions()));

  min_property_ = new rviz_common::properties::FloatProperty(
    "Min Value", 0.0, "Value which will be displayed as black.", this,
    SLOT(updateNormalizeOptions()));

  max_property_ = new rviz_common::properties::FloatProperty(
    "Max Value", 1.0, "Value which will be displayed as white.", this,
    SLOT(updateNormalizeOptions()));

  median_buffer_size_property_ = new rviz_common::properties::IntProperty(
    "Median window", 5, "Window size for median filter used for computing min/max.", this,
    SLOT(updateNormalizeOptions()));

  got_float_image_ = false;
}

// Need to override this method because of the new type RosTopicMultiTypeProperty
void ImageDisplay::setTopic(const QString & topic, const QString & datatype)
{
  (void) datatype;
  topic_property_->setString(topic);
}

void ImageDisplay::onInitialize()
{
  _RosTopicDisplay::onInitialize();
  subscription_ = std::make_shared<image_transport::SubscriberFilter>();
  updateNormalizeOptions();
  setupScreenRectangle();
  setupRenderPanel();

  render_panel_->getRenderWindow()->setupSceneAfterInit(
    [this](Ogre::SceneNode * scene_node) {
      scene_node->attachObject(screen_rect_.get());
    });

  // Populate transport->message type map dynamically from installed image_transport plugins

  pluginlib::ClassLoader<image_transport::SubscriberPlugin> sub_loader(
    "image_transport", "image_transport::SubscriberPlugin");
  transport_override_property_->clearOptions();
  transport_override_property_->addOptionStd("");
  QSet<QString> message_types;
  for (const std::string & plugin_class : sub_loader.getDeclaredClasses()) {
    const std::string message_type = image_transport::get_message_type_from_manifest(
      sub_loader.getPluginManifestPath(plugin_class), plugin_class);
    const std::string transport_name = image_transport::get_transport_name_from_manifest(
      sub_loader.getPluginManifestPath(plugin_class), plugin_class);
    if (!message_type.empty()) {
      transport_override_property_->addOptionStd(transport_name);
      message_types.insert(QString::fromStdString(message_type));
    } else {
      unknown_transports_.insert(transport_name);
    }
  }
  // Update the message types to allow in the topic_property_
  auto * multi_type_property =
    dynamic_cast<rviz_common::properties::RosTopicMultiTypeProperty *>(topic_property_);
  if (multi_type_property) {
    multi_type_property->setMessageTypes(message_types);
  }
  // Register this panel for the discovered message types
  context_->updatePluginMessageTypes(this->getClassId(), message_types);
}

ImageDisplay::~ImageDisplay()
{
  unsubscribe();
}

void ImageDisplay::onEnable()
{
  subscribe();
}

void ImageDisplay::onDisable()
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
void ImageDisplay::incomingMessage(const sensor_msgs::msg::Image::ConstSharedPtr & img_msg)
{
  if (!img_msg) {
    return;
  }

  ++messages_received_;
  QString topic_str = QString::number(messages_received_) + " messages received";
  rviz_common::properties::StatusProperty::Level topic_status_level =
    rviz_common::properties::StatusProperty::Ok;
  // Append topic subscription frequency if we can lock rviz_ros_node_.
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> node_interface =
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

  processMessage(img_msg);
}


void ImageDisplay::subscribe()
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

  // Only need to do this once but setStatusStd doesn't work in onInitialize
  if (!unknown_transports_.empty()) {
    std::string transports_str;
    for (const std::string & transport : unknown_transports_) {
      transports_str += transport + ", ";
    }
    // Trim the trailing comma
    transports_str = transports_str.substr(0, transports_str.size() - 2);
    setStatusStd(rviz_common::properties::StatusProperty::Warn,
      "Unregistered image_transport Plugins", transports_str +
      "\nEnsure plugins.xml includes the message_type tag!");
  }

  // Use override property for transport hint if set, otherwise deduce from topic name
  std::string transport_hint = transport_override_property_->getStdString();
  if (transport_hint.empty()) {
    transport_hint = getTransportFromTopic(topic_property_->getStdString());
  }
  rclcpp::Node::SharedPtr node = rviz_ros_node_.lock()->get_raw_node();
  try {
    // image_transport::Subscriber only requires one callback for "raw" and the other types are
    // automatically converted.
    subscription_->subscribe(
      *node,
      getBaseTopicFromTopic(topic_property_->getTopicStd()),
      transport_hint,
      qos_profile);
    subscription_start_time_ = node->now();
    subscription_callback_ = subscription_->registerCallback(
      std::bind(
        &rviz_default_plugins::displays::ImageDisplay::incomingMessage,
        this, std::placeholders::_1));
    setStatus(rviz_common::properties::StatusProperty::Ok, "Topic", "OK");
  } catch (rclcpp::exceptions::InvalidTopicNameError & e) {
    setStatus(
      rviz_common::properties::StatusProperty::Error, "Topic",
      QString("Error subscribing: ") + e.what());
  } catch (image_transport::TransportLoadException & e) {
    setStatus(
      rviz_common::properties::StatusProperty::Error, "Topic",
      QString("Error subscribing: ") + e.what());
  }
}

void ImageDisplay::updateTopic() {resetSubscription();}

void ImageDisplay::transformerChangedCallback() {resetSubscription();}

void ImageDisplay::resetSubscription()
{
  unsubscribe();
  reset();
  subscribe();
  context_->queueRender();
}

void ImageDisplay::unsubscribe()
{
  if (subscription_) {
    subscription_->unsubscribe();
  }
}

void ImageDisplay::updateNormalizeOptions()
{
  if (got_float_image_) {
    bool normalize = normalize_property_->getBool();

    normalize_property_->setHidden(false);
    min_property_->setHidden(normalize);
    max_property_->setHidden(normalize);
    median_buffer_size_property_->setHidden(!normalize);

    texture_->setNormalizeFloatImage(
      normalize, min_property_->getFloat(), max_property_->getFloat());
    texture_->setMedianFrames(median_buffer_size_property_->getInt());
  } else {
    normalize_property_->setHidden(true);
    min_property_->setHidden(true);
    max_property_->setHidden(true);
    median_buffer_size_property_->setHidden(true);
  }
}

void ImageDisplay::clear() {texture_->clear();}

void ImageDisplay::update(std::chrono::nanoseconds wall_dt, std::chrono::nanoseconds ros_dt)
{
  (void)wall_dt;
  (void)ros_dt;
  try {
    texture_->update();

    // make sure the aspect ratio of the image is preserved
    float win_width = render_panel_->width();
    float win_height = render_panel_->height();

    float img_width = texture_->getWidth();
    float img_height = texture_->getHeight();

    if (img_width != 0 && img_height != 0 && win_width != 0 && win_height != 0) {
      float img_aspect = img_width / img_height;
      float win_aspect = win_width / win_height;

      if (img_aspect > win_aspect) {
        screen_rect_->setCorners(
          -1.0f, 1.0f * win_aspect / img_aspect, 1.0f, -1.0f * win_aspect / img_aspect, false);
      } else {
        screen_rect_->setCorners(
          -1.0f * img_aspect / win_aspect, 1.0f, 1.0f * img_aspect / win_aspect, -1.0f, false);
      }
    }
  } catch (UnsupportedImageEncoding & e) {
    setStatus(rviz_common::properties::StatusProperty::Error, "Image", e.what());
  }
}

void ImageDisplay::reset()
{
  Display::reset();
  messages_received_ = 0;
  clear();
}

/* This is called by incomingMessage(). */
void ImageDisplay::processMessage(sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  bool got_float_image =
    msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1 ||
    msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
    msg->encoding == sensor_msgs::image_encodings::TYPE_16SC1 ||
    msg->encoding == sensor_msgs::image_encodings::MONO16;

  if (got_float_image != got_float_image_) {
    got_float_image_ = got_float_image;
    updateNormalizeOptions();
  }
  texture_->addMessage(msg);
}

void ImageDisplay::setupScreenRectangle()
{
  static int count = 0;
  rviz_common::UniformStringStream ss;
  ss << "ImageDisplayObject" << count++;

  screen_rect_ = std::make_unique<Ogre::Rectangle2D>(true);
  screen_rect_->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY - 1);
  screen_rect_->setCorners(-1.0f, 1.0f, 1.0f, -1.0f);

  ss << "Material";
  material_ = rviz_rendering::MaterialManager::createMaterialWithNoLighting(ss.str());
  material_->setSceneBlending(Ogre::SBT_REPLACE);
  material_->setDepthWriteEnabled(false);
  material_->setDepthCheckEnabled(false);

  Ogre::TextureUnitState * tu = material_->getTechnique(0)->getPass(0)->createTextureUnitState();
  tu->setTextureName(texture_->getName());
  tu->setTextureFiltering(Ogre::TFO_NONE);
  tu->setTextureAddressingMode(Ogre::TextureUnitState::TAM_CLAMP);

  material_->setCullingMode(Ogre::CULL_NONE);
  Ogre::AxisAlignedBox aabInf;
  aabInf.setInfinite();
  screen_rect_->setBoundingBox(aabInf);
  screen_rect_->setMaterial(material_);
}

void ImageDisplay::setupRenderPanel()
{
  render_panel_ = std::make_unique<rviz_common::RenderPanel>();
  render_panel_->resize(640, 480);
  render_panel_->initialize(context_);
  setAssociatedWidget(render_panel_.get());

  static int count = 0;
  render_panel_->getRenderWindow()->setObjectName(
    "ImageDisplayRenderWindow" + QString::number(count++));
}

}  // namespace displays
}  // namespace rviz_default_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::ImageDisplay, rviz_common::Display)
