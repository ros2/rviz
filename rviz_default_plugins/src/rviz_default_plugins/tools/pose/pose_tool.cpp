/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

#include "rviz_default_plugins/tools/pose/pose_tool.hpp"

#include <memory>
#include <utility>

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# pragma GCC diagnostic ignored "-Wpedantic"
#else
#pragma warning(push)
#pragma warning(disable : 4996)
#endif

#include <OgrePlane.h>
#include <OgreRay.h>
#include <OgreSceneNode.h>
#include <OgreViewport.h>

#ifndef _WIN32
# pragma GCC diagnostic pop
#else
# pragma warning(pop)
#endif

#include "rviz_rendering/render_window.hpp"
#include "rviz_rendering/geometry.hpp"
#include "rviz_rendering/objects/arrow.hpp"
#include "rviz_common/viewport_mouse_event.hpp"
#include "rviz_common/load_resource.hpp"
#include "rviz_common/render_panel.hpp"

namespace rviz_default_plugins
{
namespace tools
{
PoseTool::PoseTool()
: rviz_common::Tool(), arrow_(nullptr)
{
  projection_finder_ = std::make_shared<rviz_rendering::ViewportProjectionFinder>();
}

PoseTool::~PoseTool()
{
  delete arrow_;
}

void PoseTool::onInitialize()
{
  arrow_ = new rviz_rendering::Arrow(scene_manager_, nullptr, 2.0f, 0.2f, 0.5f, 0.35f);
  arrow_->setColor(0.0f, 1.0f, 0.0f, 1.0f);
  arrow_->getSceneNode()->setVisible(false);
}

void PoseTool::activate()
{
  setStatus("Click and drag mouse to set position/orientation.");
  state_ = Position;
}

void PoseTool::deactivate()
{
  arrow_->getSceneNode()->setVisible(false);
}

int PoseTool::processMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  int flags = 0;

  auto point_projection_on_xy_plane = projection_finder_->getViewportPointProjectionOnXYPlane(
    event.panel->getRenderWindow(), event.x, event.y);

  if (event.leftDown()) {
    assert(state_ == Position);
    if (point_projection_on_xy_plane.first) {
      pos_ = point_projection_on_xy_plane.second;
      arrow_->setPosition(pos_);

      state_ = Orientation;
      flags |= Render;
    }
  } else if (event.type == QEvent::MouseMove && event.left()) {
    if (state_ == Orientation) {
      // compute angle in x-y plane
      if (point_projection_on_xy_plane.first) {
        auto cur_pos = point_projection_on_xy_plane.second;
        double angle = atan2(cur_pos.y - pos_.y, cur_pos.x - pos_.x);

        arrow_->getSceneNode()->setVisible(true);

        // we need base_orient, since the arrow goes along the -z axis by default
        // (for historical reasons)
        Ogre::Quaternion orient_x = Ogre::Quaternion(Ogre::Radian(-Ogre::Math::HALF_PI),
            Ogre::Vector3::UNIT_Y);

        arrow_->setOrientation(
          Ogre::Quaternion(Ogre::Radian(angle), Ogre::Vector3::UNIT_Z) * orient_x);

        flags |= Render;
      }
    }
  } else if (event.leftUp()) {
    if (state_ == Orientation) {
      // compute angle in x-y plane
      if (point_projection_on_xy_plane.first) {
        auto cur_pos = point_projection_on_xy_plane.second;
        double angle = atan2(cur_pos.y - pos_.y, cur_pos.x - pos_.x);

        onPoseSet(pos_.x, pos_.y, angle);

        flags |= (Finished | Render);
      }
    }
  }

  return flags;
}

}  // namespace tools
}  // namespace rviz_default_plugins
