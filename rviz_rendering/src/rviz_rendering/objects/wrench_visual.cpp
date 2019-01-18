/*
 * Copyright (c) 2019, Martin Idel and others
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

#include <OgreVector3.h>
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include "rviz_rendering/objects/arrow.hpp"
#include "rviz_rendering/objects/billboard_line.hpp"

#include "rviz_rendering/objects/wrench_visual.hpp"

namespace rviz_rendering
{

WrenchVisual::WrenchVisual(Ogre::SceneManager * scene_manager, Ogre::SceneNode * parent_node)
{
  scene_manager_ = scene_manager;

  // Ogre::SceneNode s form a tree, with each node storing the
  // transform (position and orientation) of itself relative to its
  // parent.  Ogre does the math of combining those transforms when it
  // is time to render.
  //
  // Here we create a node to store the pose of the WrenchStamped's header frame
  // relative to the RViz fixed frame.
  frame_node_ = parent_node->createChildSceneNode();
  force_node_ = frame_node_->createChildSceneNode();
  torque_node_ = frame_node_->createChildSceneNode();

  // We create the arrow object within the frame node so that we can
  // set its position and direction relative to its header frame.
  arrow_force_ = new rviz_rendering::Arrow(scene_manager_, force_node_);
  arrow_torque_ = new rviz_rendering::Arrow(scene_manager_, torque_node_);
  circle_torque_ = new rviz_rendering::BillboardLine(scene_manager_, torque_node_);
  circle_arrow_torque_ = new rviz_rendering::Arrow(scene_manager_, torque_node_);
}

WrenchVisual::~WrenchVisual()
{
  // Delete the arrow to make it disappear.
  delete arrow_force_;
  delete arrow_torque_;
  delete circle_torque_;
  delete circle_arrow_torque_;

  // Destroy the frame node since we don't need it anymore.
  scene_manager_->destroySceneNode(frame_node_);
}

void WrenchVisual::setWrench(const Ogre::Vector3 & force, const Ogre::Vector3 & torque)
{
  float force_length = force.length() * force_scale_;
  float torque_length = torque.length() * torque_scale_;
  // hide markers if they get too short
  bool show_force = (force_length > width_);
  bool show_torque = (torque_length > width_);

  if (show_force) {
    arrow_force_->setScale(Ogre::Vector3(force_length, width_, width_));
    arrow_force_->setDirection(force);
  }
  force_node_->setVisible(show_force);

  if (show_torque) {
    arrow_torque_->setScale(Ogre::Vector3(torque_length, width_, width_));
    arrow_torque_->setDirection(torque);
    Ogre::Vector3 axis_z(0, 0, 1);
    Ogre::Quaternion orientation = axis_z.getRotationTo(torque);
    if (std::isnan(orientation.x) ||
      std::isnan(orientation.y) ||
      std::isnan(orientation.z) ) {orientation = Ogre::Quaternion::IDENTITY;}
    // circle_arrow_torque_->setScale(Ogre::Vector3(width_, width_, 0.05));
    circle_arrow_torque_->set(0, width_ * 0.1f, width_ * 0.1f * 1.0f, width_ * 0.1f * 2.0f);
    circle_arrow_torque_->setDirection(orientation * Ogre::Vector3(0, 1, 0));
    circle_arrow_torque_->setPosition(orientation *
      Ogre::Vector3(torque_length / 4, 0, torque_length / 2));
    circle_torque_->clear();
    circle_torque_->setLineWidth(width_ * 0.05f);
    for (int i = 4; i <= 32; i++) {
      Ogre::Vector3 point = Ogre::Vector3(
        static_cast<float>((torque_length / 4) * cos(i * 2 * M_PI / 32)),
        static_cast<float>((torque_length / 4) * sin(i * 2 * M_PI / 32)),
        torque_length / 2);
      circle_torque_->addPoint(orientation * point);
    }
  }
  torque_node_->setVisible(show_torque);
}

// Position and orientation are passed through to the SceneNode.
void WrenchVisual::setFramePosition(const Ogre::Vector3 & position)
{
  frame_node_->setPosition(position);
}

void WrenchVisual::setFrameOrientation(const Ogre::Quaternion & orientation)
{
  frame_node_->setOrientation(orientation);
}

// Color is passed through to the rviz object.
void WrenchVisual::setForceColor(float r, float g, float b, float a)
{
  arrow_force_->setColor(r, g, b, a);
}
// Color is passed through to the rviz object.
void WrenchVisual::setTorqueColor(float r, float g, float b, float a)
{
  arrow_torque_->setColor(r, g, b, a);
  circle_torque_->setColor(r, g, b, a);
  circle_arrow_torque_->setColor(r, g, b, a);
}

void WrenchVisual::setForceScale(float s)
{
  force_scale_ = s;
}

void WrenchVisual::setTorqueScale(float s)
{
  torque_scale_ = s;
}

void WrenchVisual::setWidth(float w)
{
  width_ = w;
}

void WrenchVisual::setVisible(bool visible)
{
  frame_node_->setVisible(visible);
}

}  // namespace rviz_rendering
