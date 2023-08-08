/*
 * Copyright (c) 2023, Open Source Robotics Foundation, Inc.
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

#define _USE_MATH_DEFINES
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include <cmath>
#include <memory>

#include "rviz_rendering/objects/arrow.hpp"
#include "rviz_rendering/objects/billboard_line.hpp"

#include "rviz_rendering/objects/screw_visual.hpp"

namespace rviz_rendering
{
ScrewVisual::ScrewVisual(Ogre::SceneManager * scene_manager, Ogre::SceneNode * parent_node)
: linear_scale_(0.0f), angular_scale_(0.0f), width_(0.0f), hide_small_values_(true),
  scene_manager_(scene_manager)
{
  // Ogre::SceneNode s form a tree, with each node storing the transform (position and orientation)
  // of itself relative to its parent. Ogre does the math of combining those transforms
  // for rendering. Here we create a node to store the pose of the screw's header
  // frame relative to the RViz fixed frame.
  frame_node_ = parent_node->createChildSceneNode();
  linear_node_ = frame_node_->createChildSceneNode();
  angular_node_ = frame_node_->createChildSceneNode();

  // We create the arrow object within the frame node so that we can
  // set its position and direction relative to its header frame.
  arrow_linear_ = std::make_unique<rviz_rendering::Arrow>(scene_manager_, linear_node_);
  arrow_angular_ = std::make_unique<rviz_rendering::Arrow>(scene_manager_, angular_node_);
  circle_angular_ = std::make_unique<rviz_rendering::BillboardLine>(scene_manager_, angular_node_);
  circle_arrow_angular_ = std::make_unique<rviz_rendering::Arrow>(scene_manager_, angular_node_);
}

ScrewVisual::~ScrewVisual()
{
  // Destroy the frame node since we don't need it anymore.
  scene_manager_->destroySceneNode(frame_node_);
}

void ScrewVisual::setScrew(const Ogre::Vector3 & linear, const Ogre::Vector3 & angular)
{
  float linear_length = linear.length() * linear_scale_;
  float angular_length = angular.length() * angular_scale_;
  // hide markers if they get too short and hide_small_values_ is activated
  // "too short" is defined as "linear_length > width_"
  bool show_linear = (linear_length > width_) || !hide_small_values_;
  bool show_angular = (angular_length > width_) || !hide_small_values_;

  if (show_linear) {
    arrow_linear_->setScale(Ogre::Vector3(linear_length, width_, width_));
    arrow_linear_->setDirection(linear);
  }
  linear_node_->setVisible(show_linear);

  if (show_angular) {
    arrow_angular_->setScale(Ogre::Vector3(angular_length, width_, width_));
    arrow_angular_->setDirection(angular);
    Ogre::Vector3 axis_z(0, 0, 1);
    Ogre::Quaternion orientation = axis_z.getRotationTo(angular);
    if (std::isnan(orientation.x) || std::isnan(orientation.y) || std::isnan(orientation.z)) {
      orientation = Ogre::Quaternion::IDENTITY;
    }
    // circle_arrow_angular_->setScale(Ogre::Vector3(width_, width_, 0.05));
    circle_arrow_angular_->set(0, width_ * 0.1f, width_ * 0.1f * 1.0f, width_ * 0.1f * 2.0f);
    circle_arrow_angular_->setDirection(orientation * Ogre::Vector3(0, 1, 0));
    circle_arrow_angular_->setPosition(
      orientation *
      Ogre::Vector3(angular_length / 4.0f, 0, angular_length / 2.0f));
    circle_angular_->clear();
    circle_angular_->setLineWidth(width_ * 0.05f);
    // create Torque Direction Circle
    // The cirlce is divided in 32 parts because it plotted using lines. We are going to plot
    // the cirle from the 4th portion to the 31. The first 4 parts are used to visualize the arrow
    const int initialCiclePortion = 4;
    const int endCirclePortion = 32;
    for (int i = initialCiclePortion; i <= endCirclePortion; i++) {
      Ogre::Vector3 point =
        Ogre::Vector3(
        static_cast<float>((angular_length / 4.0f) * cos(i * 2.0f * M_PI / 32.0f)),
        static_cast<float>((angular_length / 4.0f) * sin(i * 2.0f * M_PI / 32.0f)),
        static_cast<float>(angular_length / 2.0f));
      circle_angular_->addPoint(orientation * point);
    }
  }
  angular_node_->setVisible(show_angular);
}

// Position and orientation are passed through to the SceneNode.
void ScrewVisual::setFramePosition(const Ogre::Vector3 & position)
{
  frame_node_->setPosition(position);
}

void ScrewVisual::setFrameOrientation(const Ogre::Quaternion & orientation)
{
  frame_node_->setOrientation(orientation);
}

// Color is passed through to the rviz object.
void ScrewVisual::setLinearColor(float r, float g, float b, float a)
{
  arrow_linear_->setColor(r, g, b, a);
}
// Color is passed through to the rviz object.
void ScrewVisual::setAngularColor(float r, float g, float b, float a)
{
  arrow_angular_->setColor(r, g, b, a);
  circle_angular_->setColor(r, g, b, a);
  circle_arrow_angular_->setColor(r, g, b, a);
}

void ScrewVisual::setLinearScale(float s)
{
  linear_scale_ = s;
}

void ScrewVisual::setAngularScale(float s)
{
  angular_scale_ = s;
}

void ScrewVisual::setWidth(float w)
{
  width_ = w;
}

void ScrewVisual::setHideSmallValues(bool h)
{
  hide_small_values_ = h;
}


void ScrewVisual::setVisible(bool visible)
{
  frame_node_->setVisible(visible);
}

}  // end namespace rviz_rendering
