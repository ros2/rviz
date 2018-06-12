/*
 * Copyright (c) 2013, Willow Garage, Inc.
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

#ifndef RVIZ_DEFAULT_PLUGINS__TOOLS__POINT__POINT_TOOL_HPP_
#define RVIZ_DEFAULT_PLUGINS__TOOLS__POINT__POINT_TOOL_HPP_

#include <OgreVector3.h>

#include <QCursor>  // NOLINT cpplint cannot handle the include order here
#include <QObject>  // NOLINT cpplint cannot handle the include order here

#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rviz_common/tool.hpp"

#include "rviz_default_plugins/visibility_control.hpp"

namespace rviz_common
{
namespace properties
{
class StringProperty;
class BoolProperty;
}
}

namespace rviz_default_plugins
{
namespace tools
{

//! The Point Tool allows the user to click on a point which
//! gets published as a PointStamped message.
class RVIZ_DEFAULT_PLUGINS_PUBLIC PointTool : public rviz_common::Tool
{
  Q_OBJECT

public:
  PointTool();

  void onInitialize() override;

  void activate() override;
  void deactivate() override;

  int processMouseEvent(rviz_common::ViewportMouseEvent & event) override;

public Q_SLOTS:
  void updateTopic();
  void updateAutoDeactivate();

protected:
  QCursor std_cursor_;
  QCursor hit_cursor_;

  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_;

  rviz_common::properties::StringProperty * topic_property_;
  rviz_common::properties::BoolProperty * auto_deactivate_property_;
  void publishPosition(const Ogre::Vector3 & position) const;
  void setStatusForPosition(const Ogre::Vector3 & position);
};

}  // namespace tools
}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__TOOLS__POINT__POINT_TOOL_HPP_
