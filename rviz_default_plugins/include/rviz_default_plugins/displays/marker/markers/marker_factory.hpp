/*
 * Copyright (c) 2018, Bosch Software Innovations GmbH.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its contributors may be used
 *   to endorse or promote products derived from this software without specific
 *   prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 */

#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__MARKER__MARKERS__MARKER_FACTORY_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__MARKER__MARKERS__MARKER_FACTORY_HPP_

#include <memory>

#include "visualization_msgs/msg/marker.hpp"

#include "rviz_default_plugins/displays/marker/markers/marker_base.hpp"
#include "rviz_default_plugins/displays/marker/marker_common.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

namespace rviz_default_plugins
{
namespace displays
{
namespace markers
{

/// Factory for markers
/**
 * \class MarkerFactory
 *
 * Factory class for markers. The marker type is choosen according the the type of the marker
 * message.
 * Markers come in as visualization_msgs::msg::Marker messages.
 * See the Marker message for more information.
*/
class RVIZ_DEFAULT_PLUGINS_PUBLIC MarkerFactory
{
public:
  /// Initialization of the marker factory
  /**
   * Initializes the marker factory. Needs to be called before the createMarkerForType method is
   * called.
   * \param owner Owning display for all created markers.
   * \param context Display context.
   * \param parent_node Ogre parent scene node.
   */
  void initialize(
    MarkerCommon * owner, rviz_common::DisplayContext * context,
    Ogre::SceneNode * parent_node);

  /// Creates a new marker.
  /**
   * Generates a marker in form of a Ogre scene node for a marker type.
   * \param marker_type marker type of the marker message
   * \return shared pointer of the generated marker
   */
  std::shared_ptr<MarkerBase>
  createMarkerForType(visualization_msgs::msg::Marker::_type_type marker_type);

private:
  MarkerCommon * owner_;
  rviz_common::DisplayContext * context_;
  Ogre::SceneNode * parent_node_;
};

}  // namespace markers
}  // namespace displays
}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__MARKER__MARKERS__MARKER_FACTORY_HPP_
