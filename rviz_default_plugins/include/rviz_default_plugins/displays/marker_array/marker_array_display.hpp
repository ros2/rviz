/*
 * Copyright (c) 2011, Willow Garage, Inc.
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

#include "../marker/marker_display.hpp"

#include "visualization_msgs/msg/marker_array.hpp"

namespace rviz_default_plugins
{
namespace displays
{

/**
 * @brief Display for an array of markers.  The MarkerDisplay class handles
 * MarkerArray messages.  This is just a wrapper to let MarkerArray
 * topics get selected in the topic browser.
 */
class MarkerArrayDisplay: public MarkerDisplay
{
Q_OBJECT
public:
  MarkerArrayDisplay();

protected:
  void onInitialize() override;

  /** @brief Overridden from MarkerDisplay. Subscribes to the marker array topic. */
  void subscribe() override;
private:
  void handleMarkerArray(visualization_msgs::msg::MarkerArray::ConstSharedPtr array);
};

}  // end namespace displays
}  // end namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__MARKER_ARRAY__MARKER_ARRAY_DISPLAY_HPP_
