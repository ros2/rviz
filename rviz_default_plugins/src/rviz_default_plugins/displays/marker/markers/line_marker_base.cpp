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

#include "rviz_default_plugins/displays/marker/markers/line_marker_base.hpp"

#include <memory>
#include <vector>
#include <string>

#include <OgreVector.h>
#include <OgreQuaternion.h>
#include <OgreSceneNode.h>

#include "rviz_rendering/objects/billboard_line.hpp"
#include "rviz_common/display_context.hpp"

#include "rviz_default_plugins/displays/marker/marker_common.hpp"
#include "rviz_default_plugins/displays/marker/markers/marker_selection_handler.hpp"

namespace rviz_default_plugins
{
namespace displays
{
namespace markers
{

LineMarkerBase::LineMarkerBase(
  MarkerCommon * owner, rviz_common::DisplayContext * context, Ogre::SceneNode * parent_node)
: MarkerBase(owner, context, parent_node), lines_(nullptr), has_per_point_color_(false)
{}

void LineMarkerBase::onNewMessage(
  const MarkerConstSharedPtr & old_message, const MarkerConstSharedPtr & new_message)
{
  (void) old_message;

  if (!lines_) {
    lines_ = std::make_shared<rviz_rendering::BillboardLine>(
      context_->getSceneManager(), scene_node_);
    handler_ = rviz_common::interaction::createSelectionHandler<MarkerSelectionHandler>(
      this, MarkerID(new_message->ns, new_message->id), context_);
    handler_->addTrackedObjects(lines_->getSceneNode());
  }

  Ogre::Vector3 pos, scale;
  Ogre::Quaternion orient;
  if (!transform(new_message, pos, orient, scale)) {  // NOLINT: is super class method
    scene_node_->setVisible(false);
    return;
  }
  scene_node_->setVisible(true);

  setPosition(pos);
  setOrientation(orient);
  lines_->setScale(scale);
  lines_->setColor(
    new_message->color.r, new_message->color.g, new_message->color.b, new_message->color.a);

  lines_->clear();

  if (new_message->points.empty()) {
    return;
  }

  if (additionalConstraintsAreNotMet(new_message)) {
    return;
  }

  lines_->setLineWidth(new_message->scale.x);
  has_per_point_color_ = new_message->colors.size() == new_message->points.size();

  convertNewMessageToBillboardLine(new_message);
}

void LineMarkerBase::addPoint(
  const MarkerBase::MarkerConstSharedPtr & new_message, size_t point_number) const
{
  const geometry_msgs::msg::Point & p = new_message->points[point_number];

  Ogre::ColourValue c = has_per_point_color_ ?
    setColor(new_message->colors[point_number]) :
    setColor(new_message->color);

  Ogre::Vector3 v(p.x, p.y, p.z);
  lines_->addPoint(v, c);
}

Ogre::ColourValue LineMarkerBase::setColor(const std_msgs::msg::ColorRGBA & color) const
{
  Ogre::ColourValue c;
  c.r = color.r;
  c.g = color.g;
  c.b = color.b;
  c.a = color.a;
  return c;
}

S_MaterialPtr LineMarkerBase::getMaterials()
{
  S_MaterialPtr materials;
  materials.insert(lines_->getMaterial());
  return materials;
}

}  // namespace markers
}  // namespace displays
}  // namespace rviz_default_plugins
