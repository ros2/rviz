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

#include "ros_image_texture_iface.hpp"

#include <deque>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# pragma GCC diagnostic ignored "-Wpedantic"
#endif
#include <OgreTexture.h>
#include <OgreImage.h>
#include <OgreSharedPtr.h>
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "sensor_msgs/msg/image.hpp"


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

struct ImageData
{
  ImageData(std::string encoding, const uint8_t * data_ptr, size_t size);

  std::string encoding_;
  Ogre::PixelFormat pixel_format_;
  const uint8_t * data_ptr_;
  size_t size_;
};

class ROSImageTexture : public ROSImageTextureIface
{
public:
  ROSImageTexture();
  ~ROSImageTexture() override;

  void addMessage(sensor_msgs::msg::Image::ConstSharedPtr image) override;
  bool update() override;
  void clear() override;

  const Ogre::String getName() override {return texture_->getName();}
  const Ogre::TexturePtr & getTexture() override {return texture_;}
  const sensor_msgs::msg::Image::ConstSharedPtr getImage() override;

  uint32_t getWidth() override {return width_;}
  uint32_t getHeight() override {return height_;}

  // automatic range normalization
  void setNormalizeFloatImage(bool normalize) override;
  void setNormalizeFloatImage(bool normalize, double min, double max) override;
  void setMedianFrames(unsigned median_frames) override;

private:
  template<typename T>
  std::vector<uint8_t> normalize(const T * image_data, size_t image_data_size);
  template<typename T>
  std::vector<uint8_t> createNewNormalizedBuffer(
    const T * image_data, size_t image_data_size, T minValue, T maxValue) const;
  double computeMedianOfSeveralFrames(std::deque<double> & buffer, double new_value);
  void updateBuffer(std::deque<double> & buffer, double value) const;
  double computeMedianOfBuffer(const std::deque<double> & buffer) const;
  template<typename T>
  void getMinimalAndMaximalValueToNormalize(
    const T * image_data, size_t image_data_size, T & minValue, T & maxValue);

  bool fillWithCurrentImage(sensor_msgs::msg::Image::ConstSharedPtr & image);
  ImageData setFormatAndNormalizeDataIfNecessary(ImageData image_data);
  void loadImageToOgreImage(const ImageData & image_data, Ogre::Image & ogre_image) const;

  sensor_msgs::msg::Image::ConstSharedPtr current_image_;
  std::mutex mutex_;
  bool new_image_;

  Ogre::TexturePtr texture_;
  Ogre::Image empty_image_;

  uint32_t width_;
  uint32_t height_;

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
