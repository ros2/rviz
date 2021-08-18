/*
 * Copyright (c) 2019, Martin Idel and others
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

#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__WRENCH__WRENCH_DISPLAY_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__WRENCH__WRENCH_DISPLAY_HPP_

#include <memory>
#include <deque>

#include "geometry_msgs/msg/wrench_stamped.hpp"

#include "rviz_common/message_filter_display.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

namespace Ogre
{
class SceneNode;
}

namespace rviz_rendering
{
class WrenchVisual;
}

namespace rviz_common
{
namespace properties
{
class ColorProperty;
class FloatProperty;
class IntProperty;
}
}

namespace rviz_default_plugins
{
namespace displays
{


class RVIZ_DEFAULT_PLUGINS_PUBLIC WrenchDisplay : public
  rviz_common::MessageFilterDisplay<geometry_msgs::msg::WrenchStamped>
{
  Q_OBJECT

public:
  WrenchDisplay();

  ~WrenchDisplay() override;

  void onInitialize() override;

  void reset() override;

  void processMessage(geometry_msgs::msg::WrenchStamped::ConstSharedPtr msg) override;

private
  Q_SLOTS:
  void updateWrenchVisuals();
  void updateHistoryLength();

private:
  std::shared_ptr<rviz_rendering::WrenchVisual> createWrenchVisual(
    const geometry_msgs::msg::WrenchStamped::ConstSharedPtr & msg,
    const Ogre::Quaternion & orientation,
    const Ogre::Vector3 & position);

  std::deque<std::shared_ptr<rviz_rendering::WrenchVisual>> visuals_;

  rviz_common::properties::BoolProperty * accept_nan_values_;
  rviz_common::properties::ColorProperty * force_color_property_;
  rviz_common::properties::ColorProperty * torque_color_property_;
  rviz_common::properties::FloatProperty * alpha_property_;
  rviz_common::properties::FloatProperty * force_scale_property_;
  rviz_common::properties::FloatProperty * torque_scale_property_;
  rviz_common::properties::FloatProperty * width_property_;
  rviz_common::properties::IntProperty * history_length_property_;
};

}  // namespace displays
}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__WRENCH__WRENCH_DISPLAY_HPP_
