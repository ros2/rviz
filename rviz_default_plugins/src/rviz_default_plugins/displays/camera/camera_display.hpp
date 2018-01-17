/*
 * Copyright (c) 2009, Willow Garage, Inc.
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

#ifndef RVIZ_CAMERA_DISPLAY_H
#define RVIZ_CAMERA_DISPLAY_H

#include <cstdint>
#include <mutex>

#include <QObject>  // NOLINT: cpplint cannot handle the include order here

#ifndef Q_MOC_RUN
#include <OgreMaterial.h>
#include <OgreRenderTargetListener.h>
#include <OgreSharedPtr.h>

# include <sensor_msgs/msg/camera_info.hpp>

// TODO(Martin-Idel-SI): Do we need those?
//# include <message_filters/subscriber.h>
//# include <tf/message_filter.h>

# include "rviz_common/ros_topic_display.hpp"
# include "rviz_common/render_panel.hpp"
# include "../image/ros_image_texture.hpp"

#include "rviz_rendering/render_window.hpp"
#endif

namespace Ogre
{
class SceneNode;
class ManualObject;
class Rectangle2D;
class Camera;
}

namespace rviz_common
{

class QueueSizeProperty;

namespace properties
{

class EnumProperty;
class FloatProperty;
class IntProperty;
class RenderPanel;
class RosTopicProperty;
class DisplayGroupVisibilityProperty;

}  // namespace properties
}  // namespace rviz_common

namespace rviz_default_plugins {

namespace displays
{

/**
 * \class CameraDisplay
 *
 */
class CameraDisplay
  : public rviz_common::RosTopicDisplay<sensor_msgs::msg::Image> ,
    public Ogre::RenderTargetListener
{
Q_OBJECT
public:
  CameraDisplay();

  ~CameraDisplay() override;

  // Overrides from Display
  void onInitialize() override;

  void fixedFrameChanged() override;

  void update(float wall_dt, float ros_dt) override;

  void reset() override;

  // Overrides from Ogre::RenderTargetListener
  void preRenderTargetUpdate(const Ogre::RenderTargetEvent & evt) override;

  void postRenderTargetUpdate(const Ogre::RenderTargetEvent & evt) override;

  static const QString BACKGROUND;
  static const QString OVERLAY;
  static const QString BOTH;

protected:
  // overrides from Display
  void onEnable() override;

  void onDisable() override;

  void processMessage(sensor_msgs::msg::Image::ConstSharedPtr msg) override;

  ROSImageTexture texture_;
  rviz_common::RenderPanel * render_panel_;

private Q_SLOTS:

  void forceRender();

  void updateAlpha();

  virtual void updateQueueSize();

private:
  void subscribe();

  void unsubscribe();

  void caminfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg);

  bool updateCamera();

  void clear();

  // void updateStatus();

  Ogre::SceneNode * bg_scene_node_;
  Ogre::SceneNode * fg_scene_node_;

  Ogre::Rectangle2D * bg_screen_rect_;
  Ogre::MaterialPtr bg_material_;

  Ogre::Rectangle2D * fg_screen_rect_;
  Ogre::MaterialPtr fg_material_;

  // TODO(Martin-Idel-SI): See whether we still need those
//  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> caminfo_sub_;
//  tf::MessageFilter<sensor_msgs::msg::CameraInfo>* caminfo_tf_filter_;

  std::unique_ptr<rviz_common::QueueSizeProperty> queue_size_property_;

  rviz_common::properties::FloatProperty * alpha_property_;
  rviz_common::properties::EnumProperty * image_position_property_;
  rviz_common::properties::FloatProperty * zoom_property_;
  rviz_common::properties::DisplayGroupVisibilityProperty * visibility_property_;

  sensor_msgs::msg::CameraInfo::ConstSharedPtr current_caminfo_;
  std::mutex caminfo_mutex_;

  bool new_caminfo_;
  bool caminfo_ok_;
  bool force_render_;

  uint32_t vis_bit_;
};

}  // namespace displays

}  // namespace rviz_default_plugins

 #endif
