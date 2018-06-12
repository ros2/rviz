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

#include "rviz_default_plugins/displays/marker/markers/shape_marker.hpp"

#include <memory>

#include "rviz_rendering/objects/shape.hpp"
#include "rviz_common/display_context.hpp"

#include "rviz_default_plugins/displays/marker/marker_common.hpp"
#include "rviz_default_plugins/displays/marker/markers/marker_selection_handler.hpp"

namespace rviz_default_plugins
{
namespace displays
{
namespace markers
{

ShapeMarker::ShapeMarker(
  MarkerCommon * owner, rviz_common::DisplayContext * context, Ogre::SceneNode * parent_node)
: MarkerBase(owner, context, parent_node),
  shape_(nullptr)
{}

void ShapeMarker::onNewMessage(
  const MarkerConstSharedPtr & old_message, const MarkerConstSharedPtr & new_message)
{
  if (!shape_ || old_message->type != new_message->type) {
    resetShapeForMessage(new_message);
  }

  Ogre::Vector3 position, scale;
  Ogre::Quaternion orientation;
  if (!transform(new_message, position, orientation, scale)) {  // NOLINT: is super class method
    scene_node_->setVisible(false);
    return;
  }
  scene_node_->setVisible(true);

  if (owner_ && new_message->scale.x * new_message->scale.y * new_message->scale.z == 0.0f) {
    owner_->setMarkerStatus(
      getID(), rviz_common::properties::StatusProperty::Warn, "Scale of 0 in one of x/y/z");
  }

  auto rotate_90deg_around_x_axis = Ogre::Quaternion(Ogre::Degree(90), Ogre::Vector3(1, 0, 0));

  setPosition(position);
  setOrientation(orientation * rotate_90deg_around_x_axis);

  shape_->setScale(rotate_90deg_around_x_axis * scale);

  shape_->setColor(
    new_message->color.r, new_message->color.g, new_message->color.b, new_message->color.a);
}

S_MaterialPtr ShapeMarker::getMaterials()
{
  S_MaterialPtr materials;
  extractMaterials(shape_->getEntity(), materials);
  return materials;
}

void ShapeMarker::resetShapeForMessage(const MarkerBase::MarkerConstSharedPtr & new_message)
{
  rviz_rendering::Shape::Type shape_type = rviz_rendering::Shape::Cube;
  switch (new_message->type) {
    case visualization_msgs::msg::Marker::CUBE:
      shape_type = rviz_rendering::Shape::Cube;
      break;
    case visualization_msgs::msg::Marker::CYLINDER:
      shape_type = rviz_rendering::Shape::Cylinder;
      break;
    case visualization_msgs::msg::Marker::SPHERE:
      shape_type = rviz_rendering::Shape::Sphere;
      break;
    default:
      break;
  }
  shape_ = std::make_shared<rviz_rendering::Shape>(
    shape_type, this->context_->getSceneManager(), this->scene_node_);

  handler_ = rviz_common::interaction::createSelectionHandler<MarkerSelectionHandler>(
    this, MarkerID(new_message->ns, new_message->id), context_);
  handler_->addTrackedObjects(shape_->getRootNode());
}

}  // namespace markers
}  // namespace displays
}  // namespace rviz_default_plugins
