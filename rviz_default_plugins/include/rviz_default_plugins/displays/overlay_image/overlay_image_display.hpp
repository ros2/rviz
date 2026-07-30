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

#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__OVERLAY_IMAGE__OVERLAY_IMAGE_DISPLAY_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__OVERLAY_IMAGE__OVERLAY_IMAGE_DISPLAY_HPP_

#ifndef Q_MOC_RUN

#include <OgreException.h>

#include <QString>  // NOLINT cpplint cannot handle include order here

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_set>
#include <utility>

#include <sensor_msgs/msg/image.hpp>

#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/int_property.hpp"
#include "rviz_common/ros_topic_display.hpp"
#include "rviz_default_plugins/displays/image/get_transport_from_topic.hpp"
#include "rviz_default_plugins/displays/image/ros_image_texture.hpp"
#include "rviz_default_plugins/displays/overlay/overlay_utils.hpp"
#include "rviz_default_plugins/visibility_control.hpp"
#include "image_transport/image_transport.hpp"
#include "image_transport/subscriber_filter.hpp"
#endif

namespace rviz_default_plugins
{
namespace displays
{

// Subscribes through image_transport, like the Image and Camera displays, so
// that a camera publishing only a compressed transport can be overlaid too.
class RVIZ_DEFAULT_PLUGINS_OVERLAY_PUBLIC OverlayImageDisplay
  : public rviz_common::_RosTopicDisplay
{
  Q_OBJECT

public:
  /// Constructor for testing, which skips onInitialize().
  explicit OverlayImageDisplay(rviz_common::DisplayContext * context);
  OverlayImageDisplay();
  ~OverlayImageDisplay() override;

  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;
  void reset() override;
  void update(std::chrono::nanoseconds wall_dt, std::chrono::nanoseconds ros_dt) override;

  bool isInRegion(int x, int y) const;
  void movePosition(int dx, int dy);
  void setPosition(int x, int y);
  std::pair<int, int> getPosition() const;
  /// Ogre draw order of this overlay; higher means drawn on top. 0 if not shown.
  uint16_t getZOrder() const;

  /// Size the overlay should have for `msg`, honouring Width/Height/Keep Aspect Ratio.
  /**
   * A Width or Height of -1 means "take it from the image".
   */
  std::pair<int, int> getOverlayDimensions(
    const sensor_msgs::msg::Image::ConstSharedPtr & msg) const;

protected Q_SLOTS:
  void subscribe();

protected:
  void unsubscribe();
  void updateTopic() override;
  void transformerChangedCallback() override;
  void resetSubscription();
  void setTopic(const QString & topic, const QString & datatype) override;
  void incomingMessage(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  void processMessage(sensor_msgs::msg::Image::ConstSharedPtr msg);

private:
  void redraw(
    const sensor_msgs::msg::Image::ConstSharedPtr & msg,
    bool overwrite_alpha,
    float alpha);

private Q_SLOTS:
  void updateWidth();
  void updateHeight();
  void updateLeft();
  void updateTop();
  void updateAlpha();
  void updateKeepAspectRatio();
  void updateOverwriteAlpha();

private:
  mutable std::mutex mutex_;

  OverlayObject::SharedPtr overlay_;
  std::unique_ptr<ROSImageTexture> image_texture_;

  rviz_common::properties::BoolProperty * keep_aspect_ratio_property_;
  rviz_common::properties::IntProperty * width_property_;
  rviz_common::properties::IntProperty * height_property_;
  rviz_common::properties::IntProperty * left_property_;
  rviz_common::properties::IntProperty * top_property_;
  rviz_common::properties::FloatProperty * alpha_property_;
  rviz_common::properties::BoolProperty * overwrite_alpha_property_;
  rviz_common::properties::EnumProperty * transport_override_property_;

  std::shared_ptr<image_transport::SubscriberFilter> subscription_;
  message_filters::Connection subscription_callback_;
  std::unordered_set<std::string> unknown_transports_;
  uint32_t messages_received_;

  int width_;
  int height_;
  int effective_width_;
  int effective_height_;
  int left_;
  int top_;
  float alpha_;
  bool keep_aspect_ratio_;
  bool overwrite_alpha_;

  bool is_message_available_;
  // Monotonic counters rather than a dirty flag: a message or a property change
  // that lands while redraw() is running bumps pending_seq_ past the value
  // update() captured, so the next update() still draws it.  Clearing a bool at
  // the end of update() would drop that change until the next message arrived.
  uint64_t pending_seq_;
  uint64_t rendered_seq_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_message_;
};

}  // namespace displays
}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__OVERLAY_IMAGE__OVERLAY_IMAGE_DISPLAY_HPP_
