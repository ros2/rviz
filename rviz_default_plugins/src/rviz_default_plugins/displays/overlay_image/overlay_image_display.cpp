// Copyright (c) 2026, Iori Yanokura
// Copyright (c) 2014, JSK Lab
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
//
// This file ports jsk_rviz_plugins::OverlayImageDisplay to rviz2.

#include "rviz_default_plugins/displays/overlay_image/overlay_image_display.hpp"

#include <OgreHardwarePixelBuffer.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTexture.h>

#include <QImage>
#include <QSet>
#include <QString>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <memory>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "image_transport/camera_common.hpp"
#include "image_transport/exception.hpp"
#include "image_transport/image_transport.hpp"
#include "image_transport/subscriber_plugin.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/node.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/properties/ros_topic_multi_type_property.hpp"
#include "rviz_common/uniform_string_stream.hpp"
#include "rviz_rendering/render_system.hpp"
#include "sensor_msgs/image_encodings.hpp"

namespace rviz_default_plugins
{
namespace displays
{

OverlayImageDisplay::OverlayImageDisplay()
: messages_received_(0),
  width_(128),
  height_(128),
  effective_width_(128),
  effective_height_(128),
  left_(128),
  top_(128),
  alpha_(0.8f),
  keep_aspect_ratio_(false),
  overwrite_alpha_(false),
  is_message_available_(false),
  pending_seq_(0),
  rendered_seq_(0)
{
  // The topic can carry any of the image_transport message types, so it needs
  // the multi-type property the Image display uses rather than the plain
  // RosTopicProperty _RosTopicDisplay creates. Drop that one first, otherwise
  // both show up in the Displays panel as two "Topic" rows. The QoS property
  // owns children parented to the topic property, so it has to go first.
  delete qos_profile_property_;
  delete topic_property_;

  topic_property_ = new rviz_common::properties::RosTopicMultiTypeProperty(
    "Topic", "", QSet<QString>(), "Image transport topic to subscribe to.", this,
    SLOT(updateTopic()));
  qos_profile_property_ =
    new rviz_common::properties::QosProfileProperty(topic_property_, rclcpp::QoS(5));

  transport_override_property_ = new rviz_common::properties::EnumProperty(
    "Transport Override", "", QString("By default this display uses the topic name to ") +
    QString("determine the image_transport type. If this is not possible, use this ") +
    QString("field to manually set the transport."),
    topic_property_, SLOT(subscribe()), this);

  keep_aspect_ratio_property_ = new rviz_common::properties::BoolProperty(
    "Keep Aspect Ratio", false, "Keep the aspect ratio of the original image.",
    this, SLOT(updateKeepAspectRatio()));

  width_property_ = new rviz_common::properties::IntProperty(
    "Width", 128, "Width of the overlay in pixels, or -1 for the image width.",
    this, SLOT(updateWidth()));
  width_property_->setMin(-1);

  height_property_ = new rviz_common::properties::IntProperty(
    "Height", 128, "Height of the overlay in pixels, or -1 for the image height.",
    this, SLOT(updateHeight()));
  height_property_->setMin(-1);

  left_property_ = new rviz_common::properties::IntProperty(
    "Left", 128, "Left position of the overlay in pixels.",
    this, SLOT(updateLeft()));

  top_property_ = new rviz_common::properties::IntProperty(
    "Top", 128, "Top position of the overlay in pixels.",
    this, SLOT(updateTop()));

  alpha_property_ = new rviz_common::properties::FloatProperty(
    "Alpha", 0.8f, "Alpha blending value used for images without an alpha channel.",
    this, SLOT(updateAlpha()));
  alpha_property_->setMin(0.0f);
  alpha_property_->setMax(1.0f);

  overwrite_alpha_property_ = new rviz_common::properties::BoolProperty(
    "Overwrite Alpha", false,
    "Overwrite the alpha channel with the Alpha property and ignore the source alpha.",
    this, SLOT(updateOverwriteAlpha()));
}

OverlayImageDisplay::OverlayImageDisplay(rviz_common::DisplayContext * context)
: OverlayImageDisplay()
{
  context_ = context;
  scene_manager_ = context_->getSceneManager();
  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
}

OverlayImageDisplay::~OverlayImageDisplay()
{
  unsubscribe();
}

void OverlayImageDisplay::onInitialize()
{
  _RosTopicDisplay::onInitialize();
  rviz_rendering::RenderSystem::get()->prepareOverlays(scene_manager_);
  image_texture_ = std::make_unique<ROSImageTexture>();
  subscription_ = std::make_shared<image_transport::SubscriberFilter>();

  // Populate the transport list from the installed image_transport plugins,
  // exactly as the Image display does.
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
  auto * multi_type_property =
    dynamic_cast<rviz_common::properties::RosTopicMultiTypeProperty *>(topic_property_);
  if (multi_type_property) {
    multi_type_property->setMessageTypes(message_types);
  }
  context_->updatePluginMessageTypes(this->getClassId(), message_types);

  updateWidth();
  updateHeight();
  updateLeft();
  updateTop();
  updateAlpha();
  updateKeepAspectRatio();
  updateOverwriteAlpha();
}

void OverlayImageDisplay::setTopic(const QString & topic, const QString & datatype)
{
  (void) datatype;
  topic_property_->setString(topic);
}

void OverlayImageDisplay::onEnable()
{
  subscribe();
  if (overlay_) {
    overlay_->show();
  }
}

void OverlayImageDisplay::onDisable()
{
  if (overlay_) {
    overlay_->hide();
  }
  unsubscribe();
  reset();
}

void OverlayImageDisplay::subscribe()
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

  if (!unknown_transports_.empty()) {
    std::string transports_str;
    for (const std::string & transport : unknown_transports_) {
      transports_str += transport + ", ";
    }
    transports_str = transports_str.substr(0, transports_str.size() - 2);
    setStatusStd(
      rviz_common::properties::StatusProperty::Warn,
      "Unregistered image_transport Plugins", transports_str +
      "\nEnsure plugins.xml includes the message_type tag!");
  }

  std::string transport_hint = transport_override_property_->getStdString();
  if (transport_hint.empty()) {
    transport_hint = getTransportFromTopic(topic_property_->getStdString());
  }

  auto node_interface = rviz_ros_node_.lock();
  if (!node_interface) {
    return;
  }
  rclcpp::Node::SharedPtr node = node_interface->get_raw_node();
  try {
    subscription_->subscribe(
      *node,
      getBaseTopicFromTopic(topic_property_->getTopicStd()),
      transport_hint,
      qos_profile);
    messages_received_ = 0;
    subscription_callback_ = subscription_->registerCallback(
      std::bind(&OverlayImageDisplay::incomingMessage, this, std::placeholders::_1));
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

void OverlayImageDisplay::unsubscribe()
{
  subscription_callback_.disconnect();
  if (subscription_) {
    subscription_->unsubscribe();
  }
}

void OverlayImageDisplay::updateTopic() {resetSubscription();}

void OverlayImageDisplay::transformerChangedCallback() {resetSubscription();}

void OverlayImageDisplay::resetSubscription()
{
  unsubscribe();
  reset();
  subscribe();
  if (context_) {
    context_->queueRender();
  }
}

void OverlayImageDisplay::reset()
{
  _RosTopicDisplay::reset();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_message_.reset();
    is_message_available_ = false;
    // Not a reset to zero: pending_seq_ must stay ahead of rendered_seq_ so a
    // message that arrives right after the reset is still drawn.
    ++pending_seq_;
    rendered_seq_ = pending_seq_;
  }

  if (image_texture_) {
    image_texture_->clear();
  }
}

void OverlayImageDisplay::incomingMessage(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  if (!msg) {
    return;
  }

  ++messages_received_;
  setStatus(
    rviz_common::properties::StatusProperty::Ok, "Topic",
    QString::number(messages_received_) + " messages received");

  processMessage(msg);
}

void OverlayImageDisplay::processMessage(sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  if (!msg) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_message_ = msg;
    is_message_available_ = true;
    ++pending_seq_;
  }

  if (context_) {
    context_->queueRender();
  }
}

std::pair<int, int> OverlayImageDisplay::getOverlayDimensions(
  const sensor_msgs::msg::Image::ConstSharedPtr & msg) const
{
  int width = width_;
  int height = height_;

  if (width < 0) {
    width = msg ? static_cast<int>(msg->width) : 128;
  }
  if (height < 0) {
    height = msg ? static_cast<int>(msg->height) : 128;
  }

  if (keep_aspect_ratio_ && msg && msg->width > 0U) {
    const double aspect_ratio = static_cast<double>(msg->height) / static_cast<double>(msg->width);
    height = static_cast<int>(std::ceil(static_cast<double>(width) * aspect_ratio));
  }

  return std::make_pair(width, height);
}

void OverlayImageDisplay::redraw(
  const sensor_msgs::msg::Image::ConstSharedPtr & msg,
  bool overwrite_alpha,
  float alpha)
{
  if (!image_texture_ || !overlay_) {
    return;
  }

  image_texture_->addMessage(msg);
  if (!image_texture_->update()) {
    return;
  }

  const Ogre::TexturePtr & source_texture = image_texture_->getTexture();
  if (source_texture == nullptr) {
    return;
  }

  const uint32_t texture_width = source_texture->getWidth();
  const uint32_t texture_height = source_texture->getHeight();
  if (texture_width == 0U || texture_height == 0U) {
    return;
  }

  std::vector<uint8_t> pixel_data(static_cast<size_t>(texture_width) * texture_height * 4U);
  Ogre::PixelBox source_box(
    texture_width,
    texture_height,
    1,
    Ogre::PF_A8R8G8B8,
    pixel_data.data());
  source_texture->getBuffer(0, 0)->blitToMemory(source_box);

  const bool use_fixed_alpha =
    overwrite_alpha || !sensor_msgs::image_encodings::hasAlpha(msg->encoding);
  if (use_fixed_alpha) {
    const uint8_t alpha_byte = static_cast<uint8_t>(std::clamp(alpha, 0.0f, 1.0f) * 255.0f);
    for (size_t i = 3; i < pixel_data.size(); i += 4) {
      pixel_data[i] = alpha_byte;
    }
  }

  ScopedPixelBuffer target_buffer = overlay_->getBuffer();
  QImage destination = target_buffer.getQImage(*overlay_);
  if (destination.isNull()) {
    return;
  }

  // Copy row by row: the destination may be padded to a hardware row pitch, so
  // a single memcpy of the whole block would shear the image on such a backend.
  const size_t source_stride = static_cast<size_t>(texture_width) * 4U;
  const size_t destination_stride = static_cast<size_t>(destination.bytesPerLine());
  const size_t row_bytes = std::min(source_stride, destination_stride);
  const size_t rows = std::min<size_t>(texture_height, static_cast<size_t>(destination.height()));
  for (size_t row = 0; row < rows; ++row) {
    std::memcpy(
      destination.bits() + row * destination_stride,
      pixel_data.data() + row * source_stride,
      row_bytes);
  }
}

void OverlayImageDisplay::update(
  std::chrono::nanoseconds wall_dt,
  std::chrono::nanoseconds ros_dt)
{
  (void) wall_dt;
  (void) ros_dt;

  if (!isEnabled()) {
    return;
  }

  sensor_msgs::msg::Image::ConstSharedPtr msg;
  bool is_message_available = false;
  uint64_t seq = 0;
  bool overwrite_alpha = false;
  float alpha = 0.8f;
  int left = 0;
  int top = 0;
  std::pair<int, int> dimensions(128, 128);

  {
    std::lock_guard<std::mutex> lock(mutex_);
    msg = latest_message_;
    is_message_available = is_message_available_;
    seq = pending_seq_;
    overwrite_alpha = overwrite_alpha_;
    alpha = alpha_;
    left = left_;
    top = top_;
    dimensions = getOverlayDimensions(msg);
    effective_width_ = dimensions.first;
    effective_height_ = dimensions.second;
  }

  if (!is_message_available || !msg) {
    return;
  }

  if (!overlay_) {
    static uint32_t count = 0;
    rviz_common::UniformStringStream ss;
    ss << "OverlayImageDisplayObject" << count++;
    overlay_ = std::make_shared<OverlayObject>(ss.str());
    overlay_->show();
  }

  overlay_->setDimensions(dimensions.first, dimensions.second);
  overlay_->setPosition(left, top);

  if (seq == rendered_seq_) {
    return;
  }

  overlay_->updateTextureSize(msg->width, msg->height);

  try {
    redraw(msg, overwrite_alpha, alpha);
    setStatus(rviz_common::properties::StatusProperty::Ok, "Image", "OK");
  } catch (const UnsupportedImageEncoding & exception) {
    setStatus(rviz_common::properties::StatusProperty::Error, "Image", exception.what());
  } catch (const Ogre::Exception & exception) {
    setStatus(
      rviz_common::properties::StatusProperty::Error, "Image",
      QString::fromStdString(exception.getDescription()));
  } catch (const std::exception & exception) {
    setStatus(rviz_common::properties::StatusProperty::Error, "Image", exception.what());
  }

  // Only mark the captured sequence as drawn.  Anything that arrived while
  // redraw() was running has already pushed pending_seq_ further, so it is
  // still pending and gets drawn on the next tick.
  rendered_seq_ = seq;
}

void OverlayImageDisplay::updateWidth()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    width_ = width_property_->getInt();
    ++pending_seq_;
  }
  if (context_) {
    context_->queueRender();
  }
}

void OverlayImageDisplay::updateHeight()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    height_ = height_property_->getInt();
    ++pending_seq_;
  }
  if (context_) {
    context_->queueRender();
  }
}

void OverlayImageDisplay::updateLeft()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    left_ = left_property_->getInt();
  }
  if (context_) {
    context_->queueRender();
  }
}

void OverlayImageDisplay::updateTop()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    top_ = top_property_->getInt();
  }
  if (context_) {
    context_->queueRender();
  }
}

void OverlayImageDisplay::updateAlpha()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    alpha_ = alpha_property_->getFloat();
    ++pending_seq_;
  }
  if (context_) {
    context_->queueRender();
  }
}

void OverlayImageDisplay::updateKeepAspectRatio()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    keep_aspect_ratio_ = keep_aspect_ratio_property_->getBool();
    ++pending_seq_;
  }
  height_property_->setHidden(keep_aspect_ratio_property_->getBool());
  if (context_) {
    context_->queueRender();
  }
}

void OverlayImageDisplay::updateOverwriteAlpha()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    overwrite_alpha_ = overwrite_alpha_property_->getBool();
    ++pending_seq_;
  }
  if (context_) {
    context_->queueRender();
  }
}

bool OverlayImageDisplay::isInRegion(int x, int y) const
{
  // An overlay that is not on screen must not be pickable: before the first
  // message the geometry below is only the configured default, so without this
  // check the tool would drag an invisible rectangle.
  if (!overlay_ || !overlay_->isVisible()) {
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (!is_message_available_) {
    return false;
  }

  return
    top_ < y && (top_ + effective_height_) > y &&
    left_ < x && (left_ + effective_width_) > x;
}

uint16_t OverlayImageDisplay::getZOrder() const
{
  return overlay_ ? overlay_->getZOrder() : 0U;
}

void OverlayImageDisplay::movePosition(int dx, int dy)
{
  const auto [left, top] = getPosition();
  setPosition(left + dx, top + dy);
}

void OverlayImageDisplay::setPosition(int x, int y)
{
  if (top_property_->getInt() != y) {
    top_property_->setInt(y);
  }
  if (left_property_->getInt() != x) {
    left_property_->setInt(x);
  }
}

std::pair<int, int> OverlayImageDisplay::getPosition() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return std::make_pair(left_, top_);
}

}  // namespace displays
}  // namespace rviz_default_plugins

PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::OverlayImageDisplay, rviz_common::Display)
