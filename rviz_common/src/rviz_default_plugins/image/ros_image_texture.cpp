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

#include "ros_image_texture.hpp"

#include <algorithm>
#include <deque>
#include <limits>
#include <map>
#include <mutex>
#include <sstream>
#include <iostream>
#include <string>
#include <vector>

#include <OgreTextureManager.h> // NOLINT: cpplint cannot handle include order

#include "sensor_msgs/image_encodings.hpp"

#include "rviz_common/logging.hpp"

namespace rviz_default_plugins
{

ROSImageTexture::ROSImageTexture()
: new_image_(false),
  width_(0),
  height_(0),
  median_frames_(5)
{
  empty_image_.load("no_image.png", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

  static uint32_t count = 0;
  std::stringstream ss;
  ss << "ROSImageTexture" << count++;
  texture_ = Ogre::TextureManager::getSingleton().loadImage(
    ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, empty_image_,
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
  std::unique_lock<std::mutex> lock(mutex_);

  texture_->unload();
  texture_->loadImage(empty_image_);

  new_image_ = false;
  current_image_.reset();
}

const sensor_msgs::msg::Image::ConstSharedPtr ROSImageTexture::getImage()
{
  std::unique_lock<std::mutex> lock(mutex_);

  return current_image_;
}

void ROSImageTexture::setMedianFrames(unsigned median_frames)
{
  median_frames_ = median_frames;
}

double ROSImageTexture::updateMedian(std::deque<double> & buffer, double value)
{
  // update buffer
  while (buffer.size() > median_frames_ - 1) {
    buffer.pop_back();
  }
  buffer.push_front(value);
  // get median
  std::deque<double> buffer2 = buffer;
  std::nth_element(buffer2.begin(), buffer2.begin() + buffer2.size() / 2, buffer2.end());
  return *(buffer2.begin() + buffer2.size() / 2);
}

void ROSImageTexture::setNormalizeFloatImage(bool normalize, double min, double max)
{
  normalize_ = normalize;
  min_ = min;
  max_ = max;
}

template<typename T>
std::vector<uint8_t> ROSImageTexture::normalize(
  const T * image_data, size_t image_data_size)
{
  // Prepare output buffer
  std::vector<uint8_t> buffer;
  buffer.resize(image_data_size, 0);

  T minValue;
  T maxValue;

  if (normalize_) {
    const T * input_ptr = image_data;
    // Find min. and max. pixel value
    minValue = std::numeric_limits<T>::max();
    maxValue = std::numeric_limits<T>::min();
    for (unsigned int i = 0; i < image_data_size; ++i) {
      minValue = std::min(minValue, *input_ptr);
      maxValue = std::max(maxValue, *input_ptr);
      input_ptr++;
    }

    if (median_frames_ > 1) {
      minValue = updateMedian(min_buffer_, minValue);
      maxValue = updateMedian(max_buffer_, maxValue);
    }
  } else {
    // set fixed min/max
    minValue = min_;
    maxValue = max_;
  }

  // Rescale floating point image and convert it to 8-bit
  double range = maxValue - minValue;
  if (range > 0.0) {
    const T * input_ptr = image_data;

    // Pointer to output buffer
    uint8_t * output_ptr = &buffer[0];

    // Rescale and quantize
    for (size_t i = 0; i < image_data_size; ++i, ++output_ptr, ++input_ptr) {
      double val = (static_cast<double>(*input_ptr - minValue) / range);
      if (val < 0) {val = 0;}
      if (val > 1) {val = 1;}
      *output_ptr = val * 255u;
    }
  }
  return buffer;
}

bool ROSImageTexture::update()
{
  sensor_msgs::msg::Image::ConstSharedPtr image;
  bool new_image = false;
  {
    std::unique_lock<std::mutex> lock(mutex_);

    image = current_image_;
    new_image = new_image_;
  }

  if (!image || !new_image) {
    return false;
  }

  new_image_ = false;

  if (image->data.empty()) {
    return false;
  }

  Ogre::PixelFormat format = Ogre::PF_R8G8B8;
  Ogre::Image ogre_image;
  std::vector<uint8_t> buffer;

  const uint8_t * imageDataPtr = image->data.data();
  size_t imageDataSize = image->data.size();

  // TODO(Martin-Idel-SI): Uncrustify is unable to handle the else if formatting
  /* *INDENT-OFF* */
  if (image->encoding == sensor_msgs::image_encodings::RGB8) {
    format = Ogre::PF_BYTE_RGB;
  } else if (image->encoding == sensor_msgs::image_encodings::RGBA8) {
    format = Ogre::PF_BYTE_RGBA;
  } else if (image->encoding == sensor_msgs::image_encodings::TYPE_8UC4 ||
    image->encoding == sensor_msgs::image_encodings::TYPE_8SC4 ||
    image->encoding == sensor_msgs::image_encodings::BGRA8) {
    format = Ogre::PF_BYTE_BGRA;
  } else if (image->encoding == sensor_msgs::image_encodings::TYPE_8UC3 ||
    image->encoding == sensor_msgs::image_encodings::TYPE_8SC3 ||
    image->encoding == sensor_msgs::image_encodings::BGR8) {
    format = Ogre::PF_BYTE_BGR;
  } else if (image->encoding == sensor_msgs::image_encodings::TYPE_8UC1 ||
    image->encoding == sensor_msgs::image_encodings::TYPE_8SC1 ||
    image->encoding == sensor_msgs::image_encodings::MONO8) {
    format = Ogre::PF_BYTE_L;
  } else if (image->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
    image->encoding == sensor_msgs::image_encodings::TYPE_16SC1 ||
    image->encoding == sensor_msgs::image_encodings::MONO16) {
    imageDataSize /= sizeof(uint16_t);
    buffer = normalize<uint16_t>(
      reinterpret_cast<const uint16_t *>(image->data.data()), imageDataSize);
    format = Ogre::PF_BYTE_L;
    imageDataPtr = &buffer[0];
  } else if (image->encoding.find("bayer") == 0) {
    format = Ogre::PF_BYTE_L;
  } else if (image->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    imageDataSize /= sizeof(float);
    buffer = normalize<float>(reinterpret_cast<const float *>(image->data.data()), imageDataSize);
    format = Ogre::PF_BYTE_L;
    imageDataPtr = &buffer[0];
  } else {
    throw UnsupportedImageEncoding(image->encoding);
  }
  /* *INDENT-ON* */

  width_ = image->width;
  height_ = image->height;

  // TODO(anonymous): Support different steps/strides
  Ogre::DataStreamPtr pixel_stream;
  // C-style cast is used to bypass the const modifier
  pixel_stream.reset(
    new Ogre::MemoryDataStream((uint8_t *) &imageDataPtr[0], imageDataSize));  // NOLINT

  try {
    ogre_image.loadRawData(pixel_stream, width_, height_, 1, format, 1, 0);
  } catch (Ogre::Exception & e) {
    // TODO(anonymous): signal error better
    RVIZ_COMMON_LOG_ERROR_STREAM("Error loading image: " << e.what());
    return false;
  }

  texture_->unload();
  texture_->loadImage(ogre_image);

  return true;
}

void ROSImageTexture::addMessage(sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  std::unique_lock<std::mutex> lock(mutex_);
  current_image_ = msg;
  new_image_ = true;
}

}  // namespace rviz_default_plugins
