/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * Copyright (c) 2017, Bosch Software Innovations GmbH.
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

#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__IMAGE__IMAGE_DISPLAY_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__IMAGE__IMAGE_DISPLAY_HPP_

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <memory>
# include <string>

# include <QObject>  // NOLINT cpplint cannot handle include order here

# include <OgreMaterial.h>
# include <OgreRenderTargetListener.h>
# include <OgreSharedPtr.h>

# include "rviz_common/message_filter_display.hpp"
# include "rviz_common/render_panel.hpp"
# include "rviz_common/properties/bool_property.hpp"
# include "rviz_common/properties/float_property.hpp"
# include "rviz_common/properties/int_property.hpp"

# include "rviz_default_plugins/displays/image/ros_image_texture_iface.hpp"
# include "rviz_default_plugins/visibility_control.hpp"
#include "rviz_default_plugins/displays/image/image_transport_display.hpp"
#endif


namespace Ogre
{
class SceneNode;
class Rectangle2D;
}

namespace rviz_default_plugins
{
namespace displays
{

/**
 * \class ImageDisplay
 *
 */
class RVIZ_DEFAULT_PLUGINS_PUBLIC ImageDisplay : public
  rviz_default_plugins::displays::ImageTransportDisplay<sensor_msgs::msg::Image>
{
  Q_OBJECT

public:
  explicit ImageDisplay(std::unique_ptr<ROSImageTextureIface> texture);
  ImageDisplay();
  ~ImageDisplay() override;

  // Overrides from Display
  void onInitialize() override;
  void update(float wall_dt, float ros_dt) override;
  void reset() override;

public Q_SLOTS:
  virtual void updateNormalizeOptions();

protected:
  // overrides from Display
  void onEnable() override;
  void onDisable() override;

  /* This is called by incomingMessage(). */
  void processMessage(sensor_msgs::msg::Image::ConstSharedPtr msg) override;

private:
  void setupScreenRectangle();
  void setupRenderPanel();

  void clear();

  std::unique_ptr<Ogre::Rectangle2D> screen_rect_;
  Ogre::MaterialPtr material_;

  std::unique_ptr<ROSImageTextureIface> texture_;

  std::unique_ptr<rviz_common::RenderPanel> render_panel_;

  rviz_common::properties::BoolProperty * normalize_property_;
  rviz_common::properties::FloatProperty * min_property_;
  rviz_common::properties::FloatProperty * max_property_;
  rviz_common::properties::IntProperty * median_buffer_size_property_;
  bool got_float_image_;
};

}  // namespace displays
}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__IMAGE__IMAGE_DISPLAY_HPP_
