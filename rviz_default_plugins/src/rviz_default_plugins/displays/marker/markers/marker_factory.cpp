/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

#include "marker_factory.hpp"

#include <memory>

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# pragma GCC diagnostic ignored "-Wpedantic"
#else
# pragma warning(push)
# pragma warning(disable : 4996)
#endif

#include <OgreSceneNode.h>

#ifndef _WIN32
# pragma GCC diagnostic pop
#else
# pragma warning(pop)
#endif

#include "visualization_msgs/msg/marker.hpp"

#include "rviz_common/display_context.hpp"
#include "rviz_common/logging.hpp"

#include "marker_base.hpp"
#include "arrow_marker.hpp"
#include "line_list_marker.hpp"
#include "line_strip_marker.hpp"
#include "mesh_resource_marker.hpp"
#include "points_marker.hpp"
#include "shape_marker.hpp"
#include "text_view_facing_marker.hpp"
#include "triangle_list_marker.hpp"

namespace rviz_default_plugins
{
namespace displays
{
namespace markers
{

void MarkerFactory::initialize(
  MarkerDisplay * owner, rviz_common::DisplayContext * context, Ogre::SceneNode * parent_node)
{
  owner_display_ = owner;
  context_ = context;
  parent_node_ = parent_node;
}

std::shared_ptr<MarkerBase> MarkerFactory::createMarkerForType(
  visualization_msgs::msg::Marker::_type_type marker_type)
{
  switch (marker_type) {
    case visualization_msgs::msg::Marker::CUBE:
    case visualization_msgs::msg::Marker::CYLINDER:
    case visualization_msgs::msg::Marker::SPHERE:
      return std::make_shared<ShapeMarker>(owner_display_, context_, parent_node_);

    case visualization_msgs::msg::Marker::ARROW:
      return std::make_shared<ArrowMarker>(owner_display_, context_, parent_node_);

    case visualization_msgs::msg::Marker::LINE_STRIP:
      return std::make_shared<LineStripMarker>(owner_display_, context_, parent_node_);

    case visualization_msgs::msg::Marker::LINE_LIST:
      return std::make_shared<LineListMarker>(owner_display_, context_, parent_node_);

    case visualization_msgs::msg::Marker::SPHERE_LIST:
    case visualization_msgs::msg::Marker::CUBE_LIST:
    case visualization_msgs::msg::Marker::POINTS:
      return std::make_shared<PointsMarker>(owner_display_, context_, parent_node_);

    case visualization_msgs::msg::Marker::TEXT_VIEW_FACING:
      return std::make_shared<TextViewFacingMarker>(owner_display_, context_, parent_node_);

    case visualization_msgs::msg::Marker::MESH_RESOURCE:
      return std::make_shared<MeshResourceMarker>(owner_display_, context_, parent_node_);

    case visualization_msgs::msg::Marker::TRIANGLE_LIST:
      return std::make_shared<TriangleListMarker>(owner_display_, context_, parent_node_);

    default:
      RVIZ_COMMON_LOG_ERROR_STREAM("Unknown marker type: " << marker_type);
      return nullptr;
  }
}

}  // namespace markers
}  // namespace displays
}  // namespace rviz_default_plugins
