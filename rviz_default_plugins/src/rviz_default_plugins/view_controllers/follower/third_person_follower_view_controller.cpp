/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

#include "rviz_default_plugins/view_controllers/follower/third_person_follower_view_controller.hpp"

#include <OgreQuaternion.h>
#include <OgreSceneNode.h>
#include <OgreMath.h>

#include "rviz_common/display_context.hpp"

namespace rviz_default_plugins
{
namespace view_controllers
{

void ThirdPersonFollowerViewController::updateTargetSceneNode()
{
  if (FramePositionTrackingViewController::getNewTransform()) {
    target_scene_node_->setPosition(reference_position_);

    // OGRE camera frame looks along -Z, so they call rotation around Z "roll".
    Ogre::Radian ref_yaw = reference_orientation_.getRoll(false);
    Ogre::Quaternion ref_yaw_quat(ref_yaw, Ogre::Vector3::UNIT_Z);
    target_scene_node_->setOrientation(ref_yaw_quat);

    context_->queueRender();
  }
}

}  // namespace view_controllers
}  // namespace rviz_default_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT(build/include_order)

PLUGINLIB_EXPORT_CLASS(
  rviz_default_plugins::view_controllers::ThirdPersonFollowerViewController,
  rviz_common::ViewController)
