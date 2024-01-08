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

#include "rviz_default_plugins/displays/marker/markers/marker_base.hpp"

#include <memory>
#include <string>

#include <OgreEntity.h>
#include <OgreSubEntity.h>
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreSharedPtr.h>

#include "rclcpp/rclcpp.hpp"

#include "rviz_common/display_context.hpp"
#include "rviz_common/frame_manager_iface.hpp"
#include "rviz_common/interactive_object.hpp"

#include "rviz_default_plugins/displays/marker/marker_common.hpp"
#include "rviz_default_plugins/displays/marker/markers/marker_selection_handler.hpp"

namespace rviz_default_plugins
{
namespace displays
{
namespace markers
{

MarkerBase::MarkerBase(
  MarkerCommon * owner,
  rviz_common::DisplayContext * context,
  Ogre::SceneNode * parent_node)
: owner_(owner),
  context_(context),
  scene_node_(parent_node->createChildSceneNode())
{}

MarkerBase::~MarkerBase()
{
  context_->getSceneManager()->destroySceneNode(scene_node_);
}

void MarkerBase::setMessage(const Marker & message)
{
  // copy and save to shared pointer
  MarkerConstSharedPtr message_ptr(new Marker(message));
  setMessage(message_ptr);
}

void MarkerBase::setMessage(const MarkerConstSharedPtr & message)
{
  MarkerConstSharedPtr old = message_;
  message_ = message;

  expiration_ = context_->getClock()->now() + message->lifetime;

  onNewMessage(old, message);
}

void MarkerBase::updateFrameLocked()
{
  assert(message_ && message_->frame_locked);
  onNewMessage(message_, message_);
}

bool MarkerBase::expired()
{
  return context_->getClock()->now() >= expiration_;
}

bool MarkerBase::transform(
  const MarkerConstSharedPtr & message,
  Ogre::Vector3 & pos,
  Ogre::Quaternion & orient,
  Ogre::Vector3 & scale)
{
  rclcpp::Time stamp(message->header.stamp, RCL_ROS_TIME);
  if (message->frame_locked) {
    stamp = rclcpp::Time(0, 0, context_->getClock()->get_clock_type());
  }

  if (!context_->getFrameManager()->transform(
      message->header.frame_id, stamp, message->pose, pos, orient))
  {
    std::string error;
    rclcpp::Time message_time(message->header.stamp, RCL_ROS_TIME);
    context_->getFrameManager()->transformHasProblems(
      message->header.frame_id, message_time, error);
    if (owner_) {
      owner_->setMarkerStatus(getID(), rviz_common::properties::StatusProperty::Error, error);
    }
    RVIZ_COMMON_LOG_DEBUG("Unable to transform marker message");
    return false;
  }

  scale = Ogre::Vector3(message->scale.x, message->scale.y, message->scale.z);

  return true;
}

void MarkerBase::setInteractiveObject(rviz_common::InteractiveObjectWPtr control)
{
  if (handler_) {
    handler_->setInteractiveObject(control);
  }
}

void MarkerBase::setPosition(const Ogre::Vector3 & position)
{
  scene_node_->setPosition(position);
}

void MarkerBase::setOrientation(const Ogre::Quaternion & orientation)
{
  scene_node_->setOrientation(orientation);
}

const Ogre::Vector3 & MarkerBase::getPosition()
{
  return scene_node_->getPosition();
}

const Ogre::Quaternion & MarkerBase::getOrientation()
{
  return scene_node_->getOrientation();
}

void MarkerBase::extractMaterials(Ogre::Entity * entity, S_MaterialPtr & materials)
{
  uint64_t num_sub_entities = entity->getNumSubEntities();
  for (uint64_t i = 0; i < num_sub_entities; ++i) {
    Ogre::SubEntity * sub = entity->getSubEntity(i);
    Ogre::MaterialPtr material = sub->getMaterial();
    materials.insert(material);
  }
}

}  // namespace markers
}  // namespace displays
}  // namespace rviz_default_plugins
