/*
 * Copyright (c) 2010, Willow Garage, Inc.
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

#include "rviz_default_plugins/displays/marker/markers/text_view_facing_marker.hpp"

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include "rviz_rendering/objects/movable_text.hpp"
#include "rviz_common/display_context.hpp"

#include "rviz_default_plugins/displays/marker/markers/marker_selection_handler.hpp"

namespace rviz_default_plugins
{
namespace displays
{
namespace markers
{

TextViewFacingMarker::TextViewFacingMarker(
  MarkerCommon * owner, rviz_common::DisplayContext * context, Ogre::SceneNode * parent_node)
: MarkerBase(owner, context, parent_node),
  text_(nullptr)
{}

TextViewFacingMarker::~TextViewFacingMarker()
{
  scene_node_->detachObject(text_);
  delete text_;
}

void TextViewFacingMarker::onNewMessage(
  const MarkerConstSharedPtr & old_message, const MarkerConstSharedPtr & new_message)
{
  (void) old_message;

  assert(new_message->type == visualization_msgs::msg::Marker::TEXT_VIEW_FACING);

  if (!text_) {
    text_ = new rviz_rendering::MovableText(new_message->text);
    text_->setTextAlignment(
      rviz_rendering::MovableText::H_CENTER, rviz_rendering::MovableText::V_CENTER);
    scene_node_->attachObject(text_);

    handler_ = rviz_common::interaction::createSelectionHandler<MarkerSelectionHandler>(
      this, MarkerID(new_message->ns, new_message->id), context_);
    handler_->addTrackedObject(text_);
  }

  Ogre::Vector3 pos, scale;
  Ogre::Quaternion orient;
  if (!transform(new_message, pos, orient, scale)) {  // NOLINT: is super class method
    scene_node_->setVisible(false);
    return;
  }
  scene_node_->setVisible(true);

  setPosition(pos);
  text_->setCharacterHeight(new_message->scale.z);
  text_->setColor(
    Ogre::ColourValue(
      new_message->color.r, new_message->color.g, new_message->color.b, new_message->color.a));
  text_->setCaption(new_message->text);
}

S_MaterialPtr TextViewFacingMarker::getMaterials()
{
  S_MaterialPtr materials;
  if (text_->getMaterial().get() ) {
    materials.insert(text_->getMaterial() );
  }
  return materials;
}

}  // namespace markers
}  // namespace displays
}  // namespace rviz_default_plugins
