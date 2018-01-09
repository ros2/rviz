/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

#include "frame_info.hpp"

#include <OgreSceneNode.h>

#include "rviz_rendering/arrow.hpp"
#include "rviz_rendering/axes.hpp"
#include "rviz_rendering/movable_text.hpp"
#include "rviz_common/display_context.hpp"
#include "frame_selection_handler.hpp"

namespace rviz_default_plugins
{
namespace displays
{

FrameInfo::FrameInfo(TFDisplay * display)
: display_(display),
  axes_(NULL),
  axes_coll_(0),
  parent_arrow_(NULL),
  name_text_(NULL),
  distance_to_parent_(0.0f),
  arrow_orientation_(Ogre::Quaternion::IDENTITY),
  tree_property_(NULL)
{}

void FrameInfo::updateVisibilityFromFrame()
{
  bool enabled = enabled_property_->getBool();
  selection_handler_->setEnabled(enabled);
  setEnabled(enabled);
}

void FrameInfo::updateVisibilityFromSelection()
{
  bool enabled = selection_handler_->getEnabled();
  enabled_property_->setBool(enabled);
  setEnabled(enabled);
}

void FrameInfo::setEnabled(bool enabled)
{
  if (name_node_) {
    name_node_->setVisible(display_->show_names_property_->getBool() && enabled);
  }

  if (axes_) {
    axes_->getSceneNode()->setVisible(display_->show_axes_property_->getBool() && enabled);
  }

  if (parent_arrow_) {
    if (distance_to_parent_ > 0.001f) {
      parent_arrow_->getSceneNode()->setVisible(
        display_->show_arrows_property_->getBool() && enabled);
    } else {
      parent_arrow_->getSceneNode()->setVisible(false);
    }
  }

  if (display_->all_enabled_property_->getBool() && !enabled) {
    display_->changing_single_frame_enabled_state_ = true;
    display_->all_enabled_property_->setBool(false);
    display_->changing_single_frame_enabled_state_ = false;
  }

  // Update the configuration that stores the enabled state of all frames
  display_->frame_config_enabled_state_[this->name_] = enabled;

  display_->context_->queueRender();
}

}  // namespace displays
}  // namespace rviz_default_plugins
