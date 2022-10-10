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

#include "rviz_default_plugins/displays/marker/markers/points_marker.hpp"

#include <vector>

#include "rviz_common/interaction/selection_manager.hpp"

#include "rviz_default_plugins/displays/marker/marker_common.hpp"
#include "rviz_default_plugins/displays/marker/markers/marker_selection_handler.hpp"

namespace rviz_default_plugins
{
namespace displays
{
namespace markers
{
PointsMarker::PointsMarker(
  MarkerCommon * owner, rviz_common::DisplayContext * context, Ogre::SceneNode * parent_node)
: MarkerBase(owner, context, parent_node),
  points_(nullptr)
{}

PointsMarker::~PointsMarker()
{
  scene_node_->detachObject(points_);
  delete points_;
  points_ = nullptr;
}

void PointsMarker::onNewMessage(
  const MarkerConstSharedPtr & old_message, const MarkerConstSharedPtr & new_message)
{
  (void) old_message;

  assert(
    new_message->type == visualization_msgs::msg::Marker::POINTS ||
    new_message->type == visualization_msgs::msg::Marker::CUBE_LIST ||
    new_message->type == visualization_msgs::msg::Marker::SPHERE_LIST);

  if (!points_) {
    points_ = new rviz_rendering::PointCloud();
    scene_node_->attachObject(points_);

    handler_ = rviz_common::interaction::createSelectionHandler<MarkerSelectionHandler>(
      this, MarkerID(new_message->ns, new_message->id), context_);
    points_->setPickColor(
      rviz_common::interaction::SelectionManager::handleToColor(handler_->getHandle()));
    handler_->addTrackedObject(points_);
  }

  Ogre::Vector3 pose, scale;
  Ogre::Quaternion orientation;
  if (!transform(new_message, pose, orientation, scale)) {  // NOLINT: is super class method
    scene_node_->setVisible(false);
    return;
  }
  scene_node_->setVisible(true);

  setPosition(pose);
  setOrientation(orientation);

  points_->clearAndRemoveAllPoints();

  setRenderModeAndDimensions(new_message, scale);

  if (new_message->points.empty()) {
    return;
  }

  addPointsFromMessage(new_message);
}

void PointsMarker::setRenderModeAndDimensions(
  const MarkerConstSharedPtr & new_message, Ogre::Vector3 & scale)
{
  switch (new_message->type) {
    case visualization_msgs::msg::Marker::POINTS:
      points_->setRenderMode(rviz_rendering::PointCloud::RM_SQUARES);
      points_->setDimensions(scale.x, scale.y, 0.0f);
      break;
    case visualization_msgs::msg::Marker::CUBE_LIST:
      points_->setRenderMode(rviz_rendering::PointCloud::RM_BOXES);
      points_->setDimensions(scale.x, scale.y, scale.z);
      break;
    case visualization_msgs::msg::Marker::SPHERE_LIST:
      points_->setRenderMode(rviz_rendering::PointCloud::RM_SPHERES);
      points_->setDimensions(scale.x, scale.y, scale.z);
      break;
  }
}

void PointsMarker::addPointsFromMessage(const MarkerConstSharedPtr & new_message)
{
  float red = new_message->color.r;
  float green = new_message->color.g;
  float blue = new_message->color.b;
  float alpha = new_message->color.a;

  bool has_per_point_color = new_message->colors.size() == new_message->points.size();

  bool has_nonzero_alpha = false;
  bool has_per_point_alpha = false;

  std::vector<rviz_rendering::PointCloud::Point> points;
  points.resize(new_message->points.size());

  for (size_t i = 0; i < points.size(); i++) {
    const geometry_msgs::msg::Point & message_point = new_message->points[i];
    rviz_rendering::PointCloud::Point & point_cloud_point = points[i];

    Ogre::Vector3 message_point_position(message_point.x, message_point.y, message_point.z);
    point_cloud_point.position.x = message_point_position.x;
    point_cloud_point.position.y = message_point_position.y;
    point_cloud_point.position.z = message_point_position.z;

    if (has_per_point_color) {
      const std_msgs::msg::ColorRGBA & point_color = new_message->colors[i];
      red = point_color.r;
      green = point_color.g;
      blue = point_color.b;
      alpha = point_color.a;
      has_nonzero_alpha = has_nonzero_alpha || alpha != 0.0;
      has_per_point_alpha = has_per_point_alpha || alpha != 1.0;
    }

    point_cloud_point.setColor(red, green, blue, alpha);
  }

  if (has_per_point_color) {
    if (!has_nonzero_alpha && owner_) {
      owner_->setMarkerStatus(
        getID(),
        rviz_common::properties::StatusProperty::Warn,
        "All points have a zero alpha value.");
    }
    points_->setAlpha(1.0f, has_per_point_alpha);
  } else {
    points_->setAlpha(alpha);
  }

  points_->addPoints(points.begin(), points.end());
}

void PointsMarker::setHighlightColor(float r, float g, float b)
{
  points_->setHighlightColor(r, g, b);
}

}  // namespace markers
}  // namespace displays
}  // namespace rviz_default_plugins
