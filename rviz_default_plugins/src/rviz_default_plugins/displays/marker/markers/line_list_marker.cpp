/*
 * Copyright (c) 2009, Willow Garage, Inc.
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

#include "rviz_default_plugins/displays/marker/markers/line_list_marker.hpp"

#include <vector>
#include <string>

#include <OgreVector.h>
#include <OgreSceneNode.h>

#include "rviz_rendering/objects/billboard_line.hpp"

#include "rviz_default_plugins/displays/marker/marker_common.hpp"

namespace rviz_default_plugins
{
namespace displays
{
namespace markers
{

LineListMarker::LineListMarker(
  MarkerCommon * owner, rviz_common::DisplayContext * context, Ogre::SceneNode * parent_node)
: LineMarkerBase(owner, context, parent_node)
{}

bool LineListMarker::additionalConstraintsAreNotMet(
  const MarkerBase::MarkerConstSharedPtr & new_message)
{
  if (new_message->points.size() % 2 != 0) {
    std::string error = "Line list marker [" + getStringID() + "] has an odd number of points.";
    if (owner_) {
      owner_->setMarkerStatus(getID(), rviz_common::properties::StatusProperty::Error, error);
    }
    RVIZ_COMMON_LOG_DEBUG(error);
    return true;
  }
  return false;
}

void LineListMarker::convertNewMessageToBillboardLine(const MarkerConstSharedPtr & new_message)
{
  assert(new_message->type == visualization_msgs::msg::Marker::LINE_LIST);

  lines_->setMaxPointsPerLine(2);
  lines_->setNumLines(static_cast<uint32_t>(new_message->points.size() / 2));

  for (size_t i = 0; i < new_message->points.size() / 2; i++) {
    addPoint(new_message, 2 * i);
    addPoint(new_message, 2 * i + 1);
    lines_->finishLine();
  }
}

}  // namespace markers
}  // namespace displays
}  // namespace rviz_default_plugins
