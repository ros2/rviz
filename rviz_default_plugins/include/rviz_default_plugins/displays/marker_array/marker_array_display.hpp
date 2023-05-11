/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * Copyright (c) 2018, Bosch Software Innovations GmbH.
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

#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__MARKER_ARRAY__MARKER_ARRAY_DISPLAY_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__MARKER_ARRAY__MARKER_ARRAY_DISPLAY_HPP_

#include "visualization_msgs/msg/marker_array.hpp"

#include <memory>

#include "rviz_common/ros_topic_display.hpp"

#include "rviz_default_plugins/displays/marker/marker_common.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

namespace rviz_default_plugins
{
namespace displays
{

/**
 * \class MarkerArrayDisplay
 * \brief Displays arrays of "markers" sent in by other ROS nodes on the "visualization_marker" topic
 *
 * Marker arrays come in as visualization_msgs::msg::MarkerArray messages.
 * See the Marker message for more information.
 */
class RVIZ_DEFAULT_PLUGINS_PUBLIC MarkerArrayDisplay
  : public rviz_common::RosTopicDisplay<visualization_msgs::msg::MarkerArray>
{
public:
  MarkerArrayDisplay();

  void onInitialize() override;
  void load(const rviz_common::Config & config) override;

  void update(float wall_dt, float ros_dt) override;

  void reset() override;

protected:
  void processMessage(visualization_msgs::msg::MarkerArray::ConstSharedPtr array) override;

private:
  std::unique_ptr<MarkerCommon> marker_common_;
};

}  // end namespace displays
}  // end namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__MARKER_ARRAY__MARKER_ARRAY_DISPLAY_HPP_
