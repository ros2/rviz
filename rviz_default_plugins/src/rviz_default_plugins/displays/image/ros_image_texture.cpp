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

#include "rviz_default_plugins/displays/image/ros_image_texture.hpp"

#include <algorithm>
#include <deque>
#include <limits>
#include <map>
#include <mutex>
#include <sstream>
#include <iostream>
#include <string>
#include <vector>
#include <utility>

#include <OgreTextureManager.h>  // NOLINT: cpplint cannot handle include order

#include "sensor_msgs/image_encodings.hpp"

#include "rviz_common/logging.hpp"
#include "rviz_common/uniform_string_stream.hpp"

namespace rviz_default_plugins
{
namespace displays
{

ROSImageTexture::ROSImageTexture()
: new_image_(false),
  width_(0),
  height_(0),
  median_frames_(5)
{
  empty_image_.load("no_image.png", "rviz_rendering");

  static uint32_t count = 0;
  rviz_common::UniformStringStream ss;
  ss << "ROSImageTexture" << count++;
  texture_ = Ogre::TextureManager::getSingleton().loadImage(
    ss.str(), "rviz_rendering", empty_image_,
    Ogre::TEX_TYPE_2D,
    0);

  setNormalizeFloatImage(true);
}

ROSImageTexture::~ROSImageTexture()
{
  current_image_.reset();
}

void ROSImageTexture::clear()
{
  std::lock_guard<std::mutex> lock(mutex_);

  texture_->unload();
  texture_->loadImage(empty_image_);

  new_image_ = false;
  current_image_.reset();
}

const sensor_msgs::msg::Image::ConstSharedPtr ROSImageTexture::getImage()
{
  std::lock_guard<std::mutex> lock(mutex_);

  return current_image_;
}

void ROSImageTexture::setMedianFrames(unsigned median_frames)
{
  median_frames_ = median_frames;
}

void ROSImageTexture::setNormalizeFloatImage(bool normalize)
{
  setNormalizeFloatImage(normalize, 0.0, 1.0);
}

void ROSImageTexture::setNormalizeFloatImage(bool normalize, double min, double max)
{
  normalize_ = normalize;
  min_ = min;
  max_ = max;
}

template<typename T>
std::vector<uint8_t>
ROSImageTexture::normalize(const T * image_data, size_t image_data_size)
{
  T minValue;
  T maxValue;

  getMinimalAndMaximalValueToNormalize(image_data, image_data_size, minValue, maxValue);

  return createNewNormalizedBuffer(image_data, image_data_size, minValue, maxValue);
}

template<typename T>
void
ROSImageTexture::getMinimalAndMaximalValueToNormalize(
  const T * image_data, size_t image_data_size,
  T & minValue, T & maxValue)
{
  if (normalize_) {
    const T * input_ptr = image_data;

    minValue = std::numeric_limits<T>::max();
    maxValue = std::numeric_limits<T>::min();
    for (unsigned int i = 0; i < image_data_size; ++i) {
      minValue = std::min(minValue, *input_ptr);
      maxValue = std::max(maxValue, *input_ptr);
      input_ptr++;
    }

    if (median_frames_ > 1) {
      minValue = static_cast<T>(computeMedianOfSeveralFrames(min_buffer_, minValue));
      maxValue = static_cast<T>(computeMedianOfSeveralFrames(max_buffer_, maxValue));
    }
  } else {
    minValue = static_cast<T>(min_);
    maxValue = static_cast<T>(max_);
  }
}

double ROSImageTexture::computeMedianOfSeveralFrames(std::deque<double> & buffer, double value)
{
  updateBuffer(buffer, value);
  return computeMedianOfBuffer(buffer);
}

void ROSImageTexture::updateBuffer(std::deque<double> & buffer, double value) const
{
  while (buffer.size() > median_frames_ - 1) {
    buffer.pop_back();
  }
  buffer.push_front(value);
}

double ROSImageTexture::computeMedianOfBuffer(const std::deque<double> & buffer) const
{
  std::deque<double> buffer2 = buffer;
  nth_element(buffer2.begin(), buffer2.begin() + buffer2.size() / 2, buffer2.end());
  return *(buffer2.begin() + buffer2.size() / 2);
}

template<typename T>
std::vector<uint8_t>
ROSImageTexture::createNewNormalizedBuffer(
  const T * image_data,
  size_t image_data_size,
  T minValue,
  T maxValue) const
{
  std::vector<uint8_t> buffer;
  buffer.resize(image_data_size, 0);
  uint8_t * output_ptr = &buffer[0];

  // Rescale floating point image and convert it to 8-bit
  double range = maxValue - minValue;
  if (range > 0.0) {
    const T * input_ptr = image_data;

    // Rescale and quantize
    for (size_t i = 0; i < image_data_size; ++i, ++output_ptr, ++input_ptr) {
      double val = (static_cast<double>(*input_ptr - minValue) / range);
      if (val < 0) {val = 0;}
      if (val > 1) {val = 1;}
      *output_ptr = static_cast<uint8_t>(val * 255u);
    }
  }
  return buffer;
}

ImageData::ImageData(std::string encoding, const uint8_t * data_ptr, size_t size)
: encoding_(std::move(encoding)),
  pixel_format_(Ogre::PixelFormat::PF_R8G8B8),
  data_ptr_(data_ptr),
  size_(size)
{
}

bool ROSImageTexture::update()
{
  sensor_msgs::msg::Image::ConstSharedPtr image;
  bool has_new_image = fillWithCurrentImage(image);

  if (!image || !has_new_image) {
    return false;
  }

  new_image_ = false;

  if (image->data.empty()) {
    return false;
  }

  ImageData image_data = setFormatAndNormalizeDataIfNecessary(
    ImageData(image->encoding, image->data.data(), image->data.size()));

  width_ = image->width;
  height_ = image->height;

  Ogre::Image ogre_image;
  try {
    loadImageToOgreImage(image_data, ogre_image);
  } catch (Ogre::Exception & e) {
    RVIZ_COMMON_LOG_ERROR_STREAM("Error loading image: " << e.what());
    return false;
  }

  texture_->unload();
  texture_->loadImage(ogre_image);

  return true;
}

bool ROSImageTexture::fillWithCurrentImage(sensor_msgs::msg::Image::ConstSharedPtr & image)
{
  std::lock_guard<std::mutex> lock(mutex_);

  image = current_image_;
  return new_image_;
}

ImageData ROSImageTexture::setFormatAndNormalizeDataIfNecessary(ImageData image_data)
{
  if (image_data.encoding_ == sensor_msgs::image_encodings::RGB8) {
    image_data.pixel_format_ = Ogre::PF_BYTE_RGB;
  } else if (image_data.encoding_ == sensor_msgs::image_encodings::RGBA8) {
    image_data.pixel_format_ = Ogre::PF_BYTE_RGBA;
  } else if (  // NOLINT enforces bracket on the same line, which makes code unreadable
    image_data.encoding_ == sensor_msgs::image_encodings::TYPE_8UC4 ||
    image_data.encoding_ == sensor_msgs::image_encodings::TYPE_8SC4 ||
    image_data.encoding_ == sensor_msgs::image_encodings::BGRA8)
  {
    image_data.pixel_format_ = Ogre::PF_BYTE_BGRA;
  } else if (  // NOLINT enforces bracket on the same line, which makes code unreadable
    image_data.encoding_ == sensor_msgs::image_encodings::TYPE_8UC3 ||
    image_data.encoding_ == sensor_msgs::image_encodings::TYPE_8SC3 ||
    image_data.encoding_ == sensor_msgs::image_encodings::BGR8)
  {
    image_data.pixel_format_ = Ogre::PF_BYTE_BGR;
  } else if (  // NOLINT enforces bracket on the same line, which makes code unreadable
    image_data.encoding_ == sensor_msgs::image_encodings::TYPE_8UC1 ||
    image_data.encoding_ == sensor_msgs::image_encodings::TYPE_8SC1 ||
    image_data.encoding_ == sensor_msgs::image_encodings::MONO8)
  {
    image_data.pixel_format_ = Ogre::PF_BYTE_L;
  } else if (  // NOLINT enforces bracket on the same line, which makes code unreadable
    image_data.encoding_ == sensor_msgs::image_encodings::TYPE_16UC1 ||
    image_data.encoding_ == sensor_msgs::image_encodings::TYPE_16SC1 ||
    image_data.encoding_ == sensor_msgs::image_encodings::MONO16)
  {
    image_data.size_ /= sizeof(uint16_t);
    std::vector<uint8_t> buffer = normalize<uint16_t>(
      reinterpret_cast<const uint16_t *>(image_data.data_ptr_),
      image_data.size_);
    image_data.pixel_format_ = Ogre::PF_BYTE_L;
    image_data.data_ptr_ = &buffer[0];
  } else if (image_data.encoding_.find("bayer") == 0) {
    image_data.pixel_format_ = Ogre::PF_BYTE_L;
  } else if (image_data.encoding_ == sensor_msgs::image_encodings::TYPE_32FC1) {
    image_data.size_ /= sizeof(float);
    std::vector<uint8_t> buffer = normalize<float>(
      reinterpret_cast<const float *>(image_data.data_ptr_),
      image_data.size_);
    image_data.pixel_format_ = Ogre::PF_BYTE_L;
    image_data.data_ptr_ = &buffer[0];
  } else {
    throw UnsupportedImageEncoding(image_data.encoding_);
  }
  return image_data;
}

void ROSImageTexture::loadImageToOgreImage(
  const ImageData & image_data,
  Ogre::Image & ogre_image) const
{
  Ogre::DataStreamPtr pixel_stream;
  // C-style cast is used to bypass the const modifier
  pixel_stream.reset(
    new Ogre::MemoryDataStream(
      (uint8_t *) &image_data.data_ptr_[0], image_data.size_));  // NOLINT
  ogre_image.loadRawData(pixel_stream, width_, height_, 1, image_data.pixel_format_, 1, 0);
}

void ROSImageTexture::addMessage(sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  current_image_ = msg;
  new_image_ = true;
}

}  // namespace displays
}  // namespace rviz_default_plugins
