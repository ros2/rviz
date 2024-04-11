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

#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__IMAGE__ROS_IMAGE_TEXTURE_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__IMAGE__ROS_IMAGE_TEXTURE_HPP_

#include "rviz_default_plugins/displays/image/ros_image_texture_iface.hpp"

#include <deque>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <OgreTexture.h>
#include <OgreImage.h>
#include <OgreSharedPtr.h>

#include "sensor_msgs/msg/image.hpp"
#include "rviz_default_plugins/visibility_control.hpp"


namespace rviz_default_plugins
{
namespace displays
{

class UnsupportedImageEncoding : public std::runtime_error
{
public:
  explicit UnsupportedImageEncoding(const std::string & encoding)
  : std::runtime_error("Unsupported image encoding [" + encoding + "]")
  {}
};

struct ImageData final
{
  ImageData(
    Ogre::PixelFormat pixformat,
    const uint8_t * data_ptr,
    size_t data_size_in_bytes,
    bool take_ownership);

  ~ImageData();

  Ogre::PixelFormat pixel_format_;
  const uint8_t * data_ptr_;
  size_t size_in_bytes_;
  // Depending on the input format of the data from the ROS message, we may or may not need to do
  // some kind of conversion.  In the case where we do *not* do a conversion, we directly use the
  // data pointer from the sensor_msgs::msg::Image::ConstSharedPtr and don't do any allocations for
  // performance reasons.  In the case where we *do* a conversion, we allocate memory and then this
  // class will take ownership of the data and will be responsible for freeing it.
  bool has_ownership_;
};

class ROSImageTexture : public ROSImageTextureIface
{
public:
  RVIZ_DEFAULT_PLUGINS_PUBLIC ROSImageTexture();
  RVIZ_DEFAULT_PLUGINS_PUBLIC ~ROSImageTexture() override;

  RVIZ_DEFAULT_PLUGINS_PUBLIC
  void addMessage(sensor_msgs::msg::Image::ConstSharedPtr image) override;

  RVIZ_DEFAULT_PLUGINS_PUBLIC
  bool update() override;

  RVIZ_DEFAULT_PLUGINS_PUBLIC
  void clear() override;

  RVIZ_DEFAULT_PLUGINS_PUBLIC
  const Ogre::String getName() const override;

  RVIZ_DEFAULT_PLUGINS_PUBLIC
  const Ogre::TexturePtr & getTexture() override;

  RVIZ_DEFAULT_PLUGINS_PUBLIC
  const sensor_msgs::msg::Image::ConstSharedPtr getImage() override;

  RVIZ_DEFAULT_PLUGINS_PUBLIC
  uint32_t getWidth() const override;

  RVIZ_DEFAULT_PLUGINS_PUBLIC
  uint32_t getHeight() const override;

  // automatic range normalization
  RVIZ_DEFAULT_PLUGINS_PUBLIC
  void setNormalizeFloatImage(bool normalize) override;

  RVIZ_DEFAULT_PLUGINS_PUBLIC
  void setNormalizeFloatImage(bool normalize, double min, double max) override;

  RVIZ_DEFAULT_PLUGINS_PUBLIC
  void setMedianFrames(unsigned median_frames) override;

private:
  template<typename T>
  void getMinimalAndMaximalValueToNormalize(
    const T * data_ptr, size_t num_elements, T & min_value, T & max_value);
  template<typename T>
  ImageData convertTo8bit(const uint8_t * data_ptr, size_t data_size_in_bytes);
  ImageData convertYUV422ToRGBData(const uint8_t * data_ptr, size_t data_size_in_bytes);
  ImageData convertYUV422_YUY2ToRGBData(const uint8_t * data_ptr, size_t data_size_in_bytes);

  ImageData setFormatAndNormalizeDataIfNecessary(
    const std::string & encoding, const uint8_t * data_ptr, size_t data_size_in_bytes);

  sensor_msgs::msg::Image::ConstSharedPtr current_image_;
  std::mutex mutex_;
  bool new_image_;

  Ogre::TexturePtr texture_;
  Ogre::Image empty_image_;

  uint32_t width_;
  uint32_t height_;
  uint32_t stride_;

  // fields for float image running median computation
  bool normalize_;
  double min_;
  double max_;
  unsigned median_frames_;
  std::deque<double> min_buffer_;
  std::deque<double> max_buffer_;
};

}  // namespace displays
}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__IMAGE__ROS_IMAGE_TEXTURE_HPP_
