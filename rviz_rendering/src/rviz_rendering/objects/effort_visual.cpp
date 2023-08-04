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
#include "rviz_rendering/objects/effort_visual.hpp"

#include <algorithm>
#include <cmath>

namespace rviz_rendering
{
EffortVisual::EffortVisual(
  Ogre::SceneManager * scene_manager, Ogre::SceneNode * parent_node, float width, float scale)
: scene_manager_(scene_manager), parent_node_(parent_node), width_(width), scale_(scale)
{
}

void EffortVisual::getRainbowColor(float value, Ogre::ColourValue & color)
{
  value = std::min(value, 1.0f);
  value = std::max(value, 0.0f);

  float h = value * 5.0f + 1.0f;
  int i = static_cast<int>(floor(h));
  float f = h - static_cast<float>(i);
  if (!(i & 1)) {
    f = 1 - f;  // if i is even
  }
  float n = 1 - f;

  if (i <= 1) {
    color[0] = n, color[1] = 0, color[2] = 1;
  } else if (i == 2) {
    color[0] = 0, color[1] = n, color[2] = 1;
  } else if (i == 3) {
    color[0] = 0, color[1] = 1, color[2] = n;
  } else if (i == 4) {
    color[0] = n, color[1] = 1, color[2] = 0;
  } else if (i >= 5) {
    color[0] = 1, color[1] = n, color[2] = 0;
  }
}

void EffortVisual::setEffort(const std::string & joint_name, double effort, double max_effort)
{
  bool enabled = effort_enabled_.insert(std::make_pair(joint_name, true)).first->second;

  // enable or disable draw
  if (enabled) {
    if (effort_circle_.count(joint_name) == 0) {
      effort_circle_[joint_name] =
        std::make_unique<rviz_rendering::BillboardLine>(scene_manager_, parent_node_);
    }
    if (effort_arrow_.count(joint_name) == 0) {
      effort_arrow_[joint_name] =
        std::make_unique<rviz_rendering::Arrow>(scene_manager_, parent_node_);
    }
    if (position_.count(joint_name) == 0) {
      position_[joint_name] = Ogre::Vector3(0.0f, 0.0f, 0.0f);
    }
    if (orientation_.count(joint_name) == 0) {
      orientation_[joint_name] = Ogre::Quaternion();
    }
  } else {
    if (effort_circle_.count(joint_name) != 0) {
      effort_circle_.erase(joint_name);
    }
    if (effort_arrow_.count(joint_name) != 0) {
      effort_arrow_.erase(joint_name);
    }
    // Note that we specifically do not erase the position_ and orientation_ here, as the user
    // may have set them via setFrame{Position,Orientation} below and we don't want to
    // forget that information.
  }

  if (!enabled) {
    return;
  }

  float effort_value;

  if (max_effort != 0.0) {
    effort_value = static_cast<float>(std::fmin(fabs(effort) / max_effort, 1.0f) + 0.05f);
  } else {
    effort_value = static_cast<float>(fabs(effort) + 0.05f);
  }

  effort_arrow_[joint_name]->set(0, width_ * 2.0f, width_ * 2.0f * 1.0f, width_ * 2.0f * 2.0f);
  if (effort > 0) {
    effort_arrow_[joint_name]->setDirection(orientation_[joint_name] * Ogre::Vector3(-1, 0, 0));
  } else {
    effort_arrow_[joint_name]->setDirection(orientation_[joint_name] * Ogre::Vector3(1, 0, 0));
  }
  effort_arrow_[joint_name]->setPosition(
    orientation_[joint_name] *
    Ogre::Vector3(0, 0.05f + effort_value * scale_ * 0.5f, 0) +
    position_[joint_name]);
  effort_circle_[joint_name]->clear();
  effort_circle_[joint_name]->setLineWidth(width_);
  for (int i = 0; i < 30; i++) {
    Ogre::Vector3 point =
      Ogre::Vector3(
      static_cast<float>((0.05f + effort_value * scale_ * 0.5f) * sin(i * 2.0f * M_PI / 32.0f)),
      static_cast<float>((0.05f + effort_value * scale_ * 0.5f) * cos(i * 2.0f * M_PI / 32.0f)),
      static_cast<float>(0));
    if (effort < 0) {
      point.x = -point.x;
    }
    effort_circle_[joint_name]->addPoint(orientation_[joint_name] * point + position_[joint_name]);
  }
  Ogre::ColourValue color;
  getRainbowColor(effort_value, color);
  effort_arrow_[joint_name]->setColor(color.r, color.g, color.b, color.a);
  effort_circle_[joint_name]->setColor(color.r, color.g, color.b, color.a);
}

void EffortVisual::setFrameEnabled(const std::string & joint_name, const bool e)
{
  effort_enabled_[joint_name] = e;
}

// Position and orientation are passed through to the SceneNode.
void EffortVisual::setFramePosition(const std::string & joint_name, const Ogre::Vector3 & position)
{
  position_[joint_name] = position;
}

void EffortVisual::setFrameOrientation(
  const std::string & joint_name, const Ogre::Quaternion & orientation)
{
  orientation_[joint_name] = orientation;
}

void EffortVisual::setWidth(float w)
{
  width_ = w;
}

void EffortVisual::setScale(float s)
{
  scale_ = s;
}

}  // namespace rviz_rendering
