/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * Copyright (c) 2018, Bosch Software Innovations GmbH.
 * Copyright (c) 2020, TNG Technology Consulting GmbH.
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

#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__CAMERA__CAMERA_DISPLAY_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__CAMERA__CAMERA_DISPLAY_HPP_

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include <QObject>  // NOLINT: cpplint cannot handle the include order here

#ifndef Q_MOC_RUN

# include <OgreMaterial.h>
# include <OgrePlatform.h>
# include <OgreRenderTargetListener.h>
# include <OgreSharedPtr.h>

# include "sensor_msgs/msg/camera_info.hpp"
# include "tf2_ros/message_filter.h"

# include "rviz_default_plugins/displays/image/image_transport_display.hpp"
# include "rviz_default_plugins/displays/image/ros_image_texture_iface.hpp"
# include "rviz_default_plugins/visibility_control.hpp"
# include "rviz_rendering/render_window.hpp"

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

class RenderPanel;

namespace properties
{
class EnumProperty;
class FloatProperty;
class IntProperty;
class RosTopicProperty;
class DisplayGroupVisibilityProperty;
}  // namespace properties
}  // namespace rviz_common

namespace rviz_default_plugins
{

namespace displays
{

struct ImageDimensions
{
  unsigned int height;
  unsigned int width;
};

/**
 * \class CameraDisplay
 *
 */
class RVIZ_DEFAULT_PLUGINS_PUBLIC CameraDisplay
  : public rviz_default_plugins::displays::ImageTransportDisplay<sensor_msgs::msg::Image>,
  public Ogre::RenderTargetListener
{
  Q_OBJECT

public:
  CameraDisplay();

  ~CameraDisplay() override;

  // Overrides from Display
  void onInitialize() override;

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

  void fixedFrameChanged() override;

  void processMessage(sensor_msgs::msg::Image::ConstSharedPtr msg) override;

private Q_SLOTS:
  void updateAlpha();

private:
  void subscribe() override;

  void unsubscribe() override;

  void createCameraInfoSubscription();

  bool updateCamera();

  void clear();

  Ogre::MaterialPtr createMaterial(std::string name) const;

  std::unique_ptr<Ogre::Rectangle2D> createScreenRectangle(
    const Ogre::AxisAlignedBox & bounding_box,
    const Ogre::MaterialPtr & material,
    Ogre::uint8 render_queue_group);

  bool timeDifferenceInExactSyncMode(
    const sensor_msgs::msg::Image::ConstSharedPtr & image, rclcpp::Time & rviz_time) const;

  void translatePosition(
    Ogre::Vector3 & position,
    sensor_msgs::msg::CameraInfo::ConstSharedPtr info,
    Ogre::Quaternion orientation);

  ImageDimensions getImageDimensions(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info) const;

  Ogre::Vector2 getZoomFromInfo(
    sensor_msgs::msg::CameraInfo::ConstSharedPtr info, ImageDimensions dimensions) const;

  Ogre::Matrix4 calculateProjectionMatrix(
    sensor_msgs::msg::CameraInfo::ConstSharedPtr info,
    ImageDimensions dimensions,
    const Ogre::Vector2 & zoom) const;

  Ogre::SceneNode * background_scene_node_;
  Ogre::SceneNode * overlay_scene_node_;

  std::unique_ptr<Ogre::Rectangle2D> background_screen_rect_;
  Ogre::MaterialPtr background_material_;

  std::unique_ptr<Ogre::Rectangle2D> overlay_screen_rect_;
  Ogre::MaterialPtr overlay_material_;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr caminfo_sub_;

  std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::Image,
    rviz_common::transformation::FrameTransformer>> tf_filter_;

  std::unique_ptr<ROSImageTextureIface> texture_;
  std::unique_ptr<rviz_common::RenderPanel> render_panel_;

  rviz_common::properties::FloatProperty * alpha_property_;
  rviz_common::properties::EnumProperty * image_position_property_;
  rviz_common::properties::FloatProperty * zoom_property_;
  rviz_common::properties::FloatProperty * far_plane_property_;
  rviz_common::properties::DisplayGroupVisibilityProperty * visibility_property_;

  sensor_msgs::msg::CameraInfo::ConstSharedPtr current_caminfo_;
  std::mutex caminfo_mutex_;

  bool new_caminfo_;
  bool caminfo_ok_;
  bool force_render_;

  uint32_t vis_bit_;

  void setupSceneNodes();

  void setupRenderPanel();
};

}  // namespace displays

}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__CAMERA__CAMERA_DISPLAY_HPP_
