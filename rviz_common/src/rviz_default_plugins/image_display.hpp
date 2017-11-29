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

#ifndef RVIZ_DEFAULT_PLUGINS__IMAGE_DISPLAY_HPP_
#define RVIZ_DEFAULT_PLUGINS__IMAGE_DISPLAY_HPP_

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <QObject>

# include <OgreMaterial.h>
# include <OgreRenderTargetListener.h>
# include <OgreSharedPtr.h>
#include <include/rviz_common/ros_topic_display.hpp>

# include "rviz_common/image/ros_image_texture.hpp"
# include "rviz_common/ros_topic_display.hpp"
# include "rviz_common/render_panel.hpp"

# include "rviz_common/properties/bool_property.hpp"
# include "rviz_common/properties/float_property.hpp"
# include "rviz_common/properties/int_property.hpp"
#endif


namespace Ogre
{
class SceneNode;
class Rectangle2D;
}

namespace rviz_default_plugins
{

/**
 * \class ImageDisplay
 *
 */
class ImageDisplay : public rviz_common::RosTopicDisplay<sensor_msgs::msg::Image>
{
  Q_OBJECT

public:
  ImageDisplay();
  virtual ~ImageDisplay();

  // Overrides from Display
  virtual void onInitialize();
  virtual void update(float wall_dt, float ros_dt);
  virtual void reset();

public Q_SLOTS:
  virtual void updateNormalizeOptions();

protected:
  // overrides from Display
  virtual void onEnable();
  virtual void onDisable();

  /* This is called by incomingMessage(). */
  void processMessage(const sensor_msgs::msg::Image::ConstSharedPtr msg) override;

private:
  void clear();
  void updateStatus();

  Ogre::Rectangle2D * screen_rect_;
  Ogre::MaterialPtr material_;

  rviz_common::ROSImageTexture texture_;

  rviz_common::RenderPanel * render_panel_;

  rviz_common::properties::BoolProperty * normalize_property_;
  rviz_common::properties::FloatProperty * min_property_;
  rviz_common::properties::FloatProperty * max_property_;
  rviz_common::properties::IntProperty * median_buffer_size_property_;
  bool got_float_image_;
};

}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__IMAGE_DISPLAY_HPP_
