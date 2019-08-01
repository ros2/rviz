/*
 * Copyright (c) 2013, others
 * Copyright (c) 2018, Bosch Software Innovations GmbH.
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

#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__POINT__POINT_STAMPED_DISPLAY_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__POINT__POINT_STAMPED_DISPLAY_HPP_

#include <deque>
#include <memory>

#include <QtCore>  // NOLINT cpplint cannot handle include order here

#include "geometry_msgs/msg/point_stamped.hpp"

// TODO(Martin-Idel-SI): Add again when available
// #include <rviz/message_filter_display.h>

#include "rviz_common/message_filter_display.hpp"

#include "rviz_default_plugins/visibility_control.hpp"

namespace Ogre
{
class SceneNode;
}

namespace rviz_rendering
{
class Shape;
}

namespace rviz_common
{
class DisplayContext;

namespace properties
{
class ColorProperty;
class FloatProperty;
class IntProperty;
}  // namespace properties
}  // namespace rviz_common

namespace rviz_default_plugins
{
namespace displays
{

class RVIZ_DEFAULT_PLUGINS_PUBLIC PointStampedDisplay
  : public rviz_common::MessageFilterDisplay<geometry_msgs::msg::PointStamped>
{
  Q_OBJECT

public:
  // TODO(Martin-Idel-SI): Exposed for testing, remove when nodes can be mocked
  explicit PointStampedDisplay(rviz_common::DisplayContext * context);

  PointStampedDisplay();

  ~PointStampedDisplay() override;

  void processMessage(geometry_msgs::msg::PointStamped::ConstSharedPtr msg) override;

protected:
  /** @brief Do initialization. Overridden from MessageFilterDisplay. */
  void onInitialize() override;

  void reset() override;

private Q_SLOTS:
  void updateColorAndAlpha();

  void onlyKeepHistoryLengthNumberOfVisuals();

private:
  void setUpProperties();
  void createNewSphereVisual(const geometry_msgs::msg::PointStamped::ConstSharedPtr & msg);

  std::deque<std::shared_ptr<rviz_rendering::Shape>> visuals_;

  rviz_common::properties::ColorProperty * color_property_;
  rviz_common::properties::FloatProperty * alpha_property_;
  rviz_common::properties::FloatProperty * radius_property_;
  rviz_common::properties::IntProperty * history_length_property_;
};

}  // namespace displays
}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__POINT__POINT_STAMPED_DISPLAY_HPP_
