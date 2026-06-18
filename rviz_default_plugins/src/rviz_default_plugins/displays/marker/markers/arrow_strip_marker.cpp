/*
 * Copyright (c) 2024, Open Source Robotics Foundation, Inc.
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

#include "rviz_default_plugins/displays/marker/markers/arrow_strip_marker.hpp"

#include <algorithm>
#include <memory>
#include <string>

#include <OgreEntity.h>
#include <OgreQuaternion.h>
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreVector.h>

#include "rviz_rendering/objects/arrow.hpp"
#include "rviz_rendering/objects/shape.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/msg_conversions.hpp"

#include "rviz_default_plugins/displays/marker/marker_common.hpp"
#include "rviz_default_plugins/displays/marker/markers/marker_selection_handler.hpp"

namespace rviz_default_plugins
{
namespace displays
{
namespace markers
{
ArrowStripMarker::ArrowStripMarker(
  MarkerCommon * owner, rviz_common::DisplayContext * context,
  Ogre::SceneNode * parent_node)
: MarkerBase(owner, context, parent_node)
{
}

void ArrowStripMarker::onNewMessage(
  const MarkerConstSharedPtr & old_message,
  const MarkerConstSharedPtr & new_message)
{
  (void) old_message;

  assert(new_message->type == visualization_msgs::msg::Marker::ARROW_STRIP);

  Ogre::Vector3 pos, scale;
  Ogre::Quaternion orient;
  if (!transform(new_message, pos, orient, scale)) {  // NOLINT: is super class method
    scene_node_->setVisible(false);
    return;
  }

  scene_node_->setVisible(true);
  setPosition(pos);
  setOrientation(orient);

  arrows_.clear();

  handler_.reset(new MarkerSelectionHandler(this, MarkerID(new_message->ns, new_message->id),
          context_));
  if (new_message->points.size() < 2) {
    std::string error = "Too few points to define an arrow strip.";
    if (owner_) {
      owner_->setMarkerStatus(getID(), rviz_common::properties::StatusProperty::Error, error);
    }
    RVIZ_COMMON_LOG_DEBUG(error);
    scene_node_->setVisible(false);
    return;
  }

  // if scale.x and scale.y are 0, then nothing is shown
  if (owner_ && (new_message->scale.x + new_message->scale.y == 0.0f)) {
    owner_->setMarkerStatus(
      getID(), rviz_common::properties::StatusProperty::Warn, "Scale of 0 in both x and y");
    return;
  }

  for (unsigned i = 0; i < new_message->points.size() - 1; i++) {
    Ogre::Vector3 start = rviz_common::pointMsgToOgre(new_message->points.at(i));
    Ogre::Vector3 end = rviz_common::pointMsgToOgre(new_message->points.at(i + 1));
    std::unique_ptr<rviz_rendering::Arrow> arrow =
      std::make_unique<rviz_rendering::Arrow>(context_->getSceneManager(), scene_node_);
    arrow->setEndpoints(start, end);
    arrow->setShaftDiameter(new_message->scale.x);
    arrow->setHeadDiameter(new_message->scale.y);
    float head_length = std::clamp<float>(new_message->scale.z, 0.0, arrow->getLength());
    if (head_length > 0.0) {
      arrow->setShaftHeadRatio(head_length - arrow->getLength(), head_length);
    } else {
      arrow->setShaftHeadRatio(3, 1);  // default 3:1 ratio from arrow.hpp
    }
    arrow->setColor(rviz_common::colorMsgToOgre(new_message->color));
    handler_->addTrackedObjects(arrow->getSceneNode());
    arrows_.push_back(std::move(arrow));
  }
}

}  // namespace markers
}  // namespace displays
}  // namespace rviz_default_plugins