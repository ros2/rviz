// Copyright (c) 2014, JSK Lab
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// This file is adapted from jsk_rviz_plugins OverlayCameraDisplay (ROS 1).

#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__OVERLAY_CAMERA__OVERLAY_CAMERA_DISPLAY_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__OVERLAY_CAMERA__OVERLAY_CAMERA_DISPLAY_HPP_

#include <cstdint>
#include <memory>
#include <string>

#include <QObject>  // NOLINT: cpplint cannot handle the include order here

#include "rviz_default_plugins/displays/camera/camera_display.hpp"
#include "rviz_default_plugins/displays/overlay/overlay_utils.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

namespace rviz_common
{
namespace properties
{
class FloatProperty;
class IntProperty;
}  // namespace properties
}  // namespace rviz_common

namespace rviz_default_plugins
{
namespace displays
{

class RVIZ_DEFAULT_PLUGINS_OVERLAY_PUBLIC OverlayCameraDisplay : public CameraDisplay
{
  Q_OBJECT

public:
  OverlayCameraDisplay();
  ~OverlayCameraDisplay() override;

  void onInitialize() override;
  void update(std::chrono::nanoseconds wall_dt, std::chrono::nanoseconds ros_dt) override;
  void reset() override;

protected:
  void onEnable() override;
  void onDisable() override;

private Q_SLOTS:
  void updateWidth();
  void updateHeight();
  void updateLeft();
  void updateTop();
  void updateTextureAlpha();

private:
  void ensureOverlay();
  bool createOrUpdateRenderTexture();
  void destroyRenderTexture();
  void redraw(Ogre::RenderTarget * render_target);

  OverlayObject::SharedPtr overlay_;
  rviz_common::properties::IntProperty * width_property_;
  rviz_common::properties::IntProperty * height_property_;
  rviz_common::properties::IntProperty * left_property_;
  rviz_common::properties::IntProperty * top_property_;
  rviz_common::properties::FloatProperty * texture_alpha_property_;

  int width_;
  int height_;
  int left_;
  int top_;
  float texture_alpha_;
  uint32_t render_texture_width_;
  uint32_t render_texture_height_;
  std::string render_texture_name_;
  Ogre::RenderTarget * render_texture_target_;
  Ogre::Viewport * render_texture_viewport_;
};

}  // namespace displays
}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__OVERLAY_CAMERA__OVERLAY_CAMERA_DISPLAY_HPP_
