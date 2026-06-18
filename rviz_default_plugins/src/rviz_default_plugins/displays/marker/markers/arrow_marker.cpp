// Copyright (c) 2009, Willow Garage, Inc.
// Copyright (c) 2018, Bosch Software Innovations GmbH.
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


#include "rviz_default_plugins/displays/marker/markers/arrow_marker.hpp"

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

ArrowMarker::ArrowMarker(
  MarkerCommon * owner, rviz_common::DisplayContext * context, Ogre::SceneNode * parent_node)
: MarkerBase(owner, context, parent_node),
  arrow_(nullptr), last_arrow_set_from_points_(false)
{}

S_MaterialPtr ArrowMarker::getMaterials()
{
  S_MaterialPtr materials;
  extractMaterials(arrow_->getHead()->getEntity(), materials);
  extractMaterials(arrow_->getShaft()->getEntity(), materials);
  return materials;
}

void ArrowMarker::onNewMessage(
  const MarkerConstSharedPtr & old_message, const MarkerConstSharedPtr & new_message)
{
  (void) old_message;

  assert(new_message->type == visualization_msgs::msg::Marker::ARROW);

  if (!arrow_) {
    arrow_ = std::make_unique<rviz_rendering::Arrow>(context_->getSceneManager(), scene_node_);
    setDefaultProportions();
    handler_ = rviz_common::interaction::createSelectionHandler<MarkerSelectionHandler>(
      this, MarkerID(new_message->ns, new_message->id), context_);
    handler_->addTrackedObjects(arrow_->getSceneNode());
  }

  if (new_message->points.size() == 1) {
    printErrorMessage();
    scene_node_->setVisible(false);
    return;
  }

  Ogre::Vector3 pos, scale;
  Ogre::Quaternion orientation;
  if (!transform(new_message, pos, orientation, scale)) {  // NOLINT: is super class method
    scene_node_->setVisible(false);
    return;
  }

  scene_node_->setVisible(true);

  setPosition(pos);
  setOrientation(orientation);

  arrow_->setColor(rviz_common::colorMsgToOgre(new_message->color));

  if (new_message->points.size() == 2) {
    setArrowFromPoints(new_message);
  } else {
    setArrow(new_message);
  }
}

void ArrowMarker::printErrorMessage()
{
  std::string error_message =
    "Arrow marker [" + getStringID() + "] only specified one point of a point to point arrow.";
  if (owner_) {
    owner_->setMarkerStatus(getID(), rviz_common::properties::StatusProperty::Error, error_message);
  }
  RVIZ_COMMON_LOG_DEBUG(error_message);
}

void ArrowMarker::setArrowFromPoints(const MarkerConstSharedPtr & message)
{
  last_arrow_set_from_points_ = true;

  Ogre::Vector3 start = rviz_common::pointMsgToOgre(message->points[0]);
  Ogre::Vector3 end = rviz_common::pointMsgToOgre(message->points[1]);
  arrow_->setEndpoints(start, end);
  arrow_->setShaftDiameter(message->scale.x);
  arrow_->setHeadDiameter(message->scale.y);
  float head_length = std::clamp<float>(message->scale.z, 0.0, arrow_->getLength());
  if (head_length > 0.0) {
    arrow_->setShaftHeadRatio(head_length - arrow_->getLength(), head_length);
  } else {
    arrow_->setShaftHeadRatio(3, 1); // default 3:1 ratio from arrow.hpp
  }

  // if scale.x and scale.y are 0, then nothing is shown
  if (owner_ && (message->scale.x + message->scale.y == 0.0f)) {
    owner_->setMarkerStatus(
      getID(), rviz_common::properties::StatusProperty::Warn, "Scale of 0 in both x and y");
  }
}

void ArrowMarker::setArrow(const MarkerConstSharedPtr & message)
{
  if (last_arrow_set_from_points_) {
    // Reset arrow to default proportions if we previously set it from points
    setDefaultProportions();
    arrow_->setPosition(Ogre::Vector3(0, 0, 0));
    last_arrow_set_from_points_ = false;
  }
  if (owner_ && (message->scale.x * message->scale.y * message->scale.z == 0.0f)) {
    owner_->setMarkerStatus(
      getID(), rviz_common::properties::StatusProperty::Warn, "Scale of 0 in one of x/y/z");
  }
  arrow_->setScale(rviz_common::vector3MsgToOgre(message->scale));
  arrow_->setDirection(Ogre::Vector3::UNIT_Z);
}

void ArrowMarker::setDefaultProportions()
{
  arrow_->setShaftLength(0.77);
  arrow_->setShaftDiameter(1);
  arrow_->setHeadLength(0.23);
  arrow_->setHeadDiameter(2);
}

}  // namespace markers
}  // namespace displays
}  // namespace rviz_default_plugins
