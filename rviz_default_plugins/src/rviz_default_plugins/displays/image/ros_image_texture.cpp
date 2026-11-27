// Copyright (c) 2009, Willow Garage, Inc.
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


#include "rviz_default_plugins/displays/image/ros_image_texture.hpp"

#include <algorithm>
#include <deque>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <iostream>
#include <string>
#include <tuple>
#include <vector>
#include <utility>

#include <cstring>

#include <OgreHardwarePixelBuffer.h>  // NOLINT: cpplint cannot handle include order
#include <OgreTextureManager.h>  // NOLINT: cpplint cannot handle include order

#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"

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
  median_frames_(5),
  smooth_scaling_(false),
  tex_smooth_(false),
  tex_width_(0),
  tex_height_(0),
  tex_format_(Ogre::PF_UNKNOWN)
{
  empty_image_.load("no_image.png", "rviz_rendering");

  static uint32_t count = 0;
  rviz_common::UniformStringStream ss;
  ss << "ROSImageTexture" << count++;
  texture_name_ = ss.str();

  loadEmpty();

  setNormalizeFloatImage(true);
}

ROSImageTexture::~ROSImageTexture()
{
  if (texture_) {
    if (auto * mgr = Ogre::TextureManager::getSingletonPtr()) {
      mgr->remove(texture_);
    }
    texture_.reset();
  }
  current_image_.reset();
}

void ROSImageTexture::clear()
{
  std::lock_guard<std::mutex> lock(mutex_);

  loadEmpty();

  new_image_ = false;
  current_image_.reset();

  // Drop median history so reset/resubscribe doesn't carry stale min/max
  // into the first frame of the new stream.
  min_buffer_.clear();
  max_buffer_.clear();
}

const Ogre::String ROSImageTexture::getName() const
{
  return texture_->getName();
}

const Ogre::TexturePtr & ROSImageTexture::getTexture()
{
  return texture_;
}

const sensor_msgs::msg::Image::ConstSharedPtr ROSImageTexture::getImage()
{
  std::lock_guard<std::mutex> lock(mutex_);

  return current_image_;
}

uint32_t ROSImageTexture::getWidth() const
{
  return width_;
}

uint32_t ROSImageTexture::getHeight() const
{
  return height_;
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

void ROSImageTexture::setSmoothScaling(bool enabled)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (smooth_scaling_ == enabled) {
    return;
  }
  smooth_scaling_ = enabled;
  if (current_image_) {
    // Re-upload the held frame on the next update() rather than overwriting
    // with the placeholder here — latched topics or paused bags would
    // otherwise lose the live image until the next publish.
    new_image_ = true;
  } else {
    loadEmpty();
  }
}

// Bytes per pixel for encodings that have a fixed linear row layout. Returns
// 0 for encodings whose per-row layout is non-trivial (YUV 4:2:2 packed, NV12
// planar) — those are handled explicitly by their converters using stride.
static size_t bytesPerPixelForEncoding(const std::string & encoding)
{
  namespace enc = sensor_msgs::image_encodings;
  if (encoding == enc::YUV422 || encoding == enc::YUV422_YUY2 ||
    encoding == enc::UYVY || encoding == enc::YUYV ||
    encoding == enc::NV12 || encoding == enc::NV21 || encoding == enc::NV24)
  {
    return 0;
  }
  try {
    return static_cast<size_t>(enc::numChannels(encoding)) *
           enc::bitDepth(encoding) / 8;
  } catch (const std::runtime_error &) {
    return 0;  // unknown encoding — skip repack
  }
}

static double
computeMedianOfSeveralFrames(std::deque<double> & buffer, double value, unsigned median_frames)
{
  while (buffer.size() > median_frames - 1) {
    buffer.pop_back();
  }
  buffer.push_front(value);

  // Compute the median
  std::deque<double> buffer2 = buffer;
  nth_element(buffer2.begin(), buffer2.begin() + buffer2.size() / 2, buffer2.end());
  return *(buffer2.begin() + buffer2.size() / 2);
}

bool ROSImageTexture::update()
{
  std::lock_guard<std::mutex> lock(mutex_);

  sensor_msgs::msg::Image::ConstSharedPtr image = current_image_;

  if (!image || !new_image_) {
    return false;
  }

  new_image_ = false;

  if (image->data.empty()) {
    return false;
  }

  width_ = image->width;
  height_ = image->height;
  stride_ = image->step;

  // If the publisher sent rows with trailing padding (step > width * bpp),
  // repack into a contiguous buffer. Ogre::Image::loadRawData and the
  // convertTo8bit rescale loop both assume packed rows — without this,
  // each row drifts right by `padding` bytes, producing a diagonal shear.
  // YUV/NV12 encodings are left alone because their converters honor stride
  // directly.
  std::vector<uint8_t> repacked;
  const uint8_t * data_ptr = image->data.data();
  size_t data_size = image->data.size();
  const size_t bpp = bytesPerPixelForEncoding(image->encoding);
  if (bpp != 0) {
    const size_t packed_row = static_cast<size_t>(width_) * bpp;
    if (stride_ > packed_row) {
      if (image->data.size() < static_cast<size_t>(stride_) * height_) {
        RVIZ_COMMON_LOG_ERROR_STREAM(
          "Image data size " << image->data.size() <<
            " smaller than step*height (" << stride_ * height_ << ")");
        return false;
      }
      repacked.resize(packed_row * height_);
      for (uint32_t y = 0; y < height_; ++y) {
        std::memcpy(
          repacked.data() + y * packed_row,
          image->data.data() + y * stride_,
          packed_row);
      }
      data_ptr = repacked.data();
      data_size = repacked.size();
      // Reflect the repack so any downstream code reading stride_ sees the
      // new (packed) layout.
      stride_ = static_cast<uint32_t>(packed_row);
    }
  }

  ImageData image_data = setFormatAndNormalizeDataIfNecessary(
    image->encoding, data_ptr, data_size);

  try {
    ensureTexture(width_, height_, image_data.pixel_format_);
    Ogre::PixelBox box(
      width_, height_, 1, image_data.pixel_format_,
      const_cast<uint8_t *>(image_data.data_ptr_));
    texture_->getBuffer(0, 0)->blitFromMemory(box);
  } catch (const Ogre::Exception & e) {
    RVIZ_COMMON_LOG_ERROR_STREAM("Error loading image: " << e.what());
    return false;
  }

  return true;
}

// Considering frequent execution, use inline
static inline std::tuple<uint8_t, uint8_t, uint8_t> pixelYUVToRGB(int y, int u, int v)
{
  int r = 0;
  int b = 0;
  int g = 0;

  // Values generated based on this formula
  // for converting YUV to RGB
  // R = Y + 1.403V'
  // G = Y + 0.344U' - 0.714V'
  // B = Y + 1.770U'

  v -= 128;
  u -= 128;

  r = y + (1403 * v) / 1000;
  g = y + (344 * u - 714 * v) / 1000;
  b = y + (1770 * u) / 1000;

  // pixel value must fit in a uint8_t
  return {
    std::clamp(r, 0, 255), std::clamp(g, 0, 255), std::clamp(b, 0, 255)
  };
}

struct yuyv
{
  uint8_t y0;
  uint8_t u;
  uint8_t y1;
  uint8_t v;
};

struct uyvy
{
  uint8_t u;
  uint8_t y0;
  uint8_t v;
  uint8_t y1;
};

// Function converts src_img from NV12 format to rgb
static void imageConvertNV12ToRGB(
  uint8_t * dst_img,
  const uint8_t * src_img,
  int dst_start_row,
  int dst_end_row,
  int dst_num_cols,
  uint32_t stride_in_bytes)
{
  const uint8_t * y_ptr = src_img;
  const uint8_t * uv_ptr = src_img + (dst_end_row * stride_in_bytes);

  for (int row = dst_start_row; row < dst_end_row; row++) {
    for (int col = 0; col < dst_num_cols; col++) {
      int y_index = row * stride_in_bytes + col;
      int uv_index = (row / 2) * stride_in_bytes + (col / 2) * 2;

      int final_y = y_ptr[y_index];
      int final_u = uv_ptr[uv_index + 0];
      int final_v = uv_ptr[uv_index + 1];

      std::tie(dst_img[0], dst_img[1], dst_img[2]) = pixelYUVToRGB(final_y, final_u, final_v);

      dst_img += 3;
    }
  }
}

// Function converts src_img from UYVY format to rgb
static void imageConvertUYVYToRGB(
  uint8_t * dst_img, uint8_t * src_img,
  int dst_start_row, int dst_end_row,
  int dst_num_cols, uint32_t stride_in_bytes)
{
  int final_y0 = 0;
  int final_u = 0;
  int final_y1 = 0;
  int final_v = 0;

  uint32_t stride_in_pixels = stride_in_bytes / 4;

  // rows in dst_img
  for (int row = dst_start_row; row < dst_end_row; row++) {
    // col iterates till num_cols / 2 since two rgb pixels processed each
    // iteration cols in dst_img
    for (int col = 0; col < dst_num_cols / 2; col++) {
      struct uyvy * src_ptr = reinterpret_cast<struct uyvy *>(src_img);
      struct uyvy * pixel = &src_ptr[col + row * stride_in_pixels];
      final_y0 = pixel->y0;
      final_u = pixel->u;
      final_y1 = pixel->y1;
      final_v = pixel->v;

      std::tie(dst_img[0], dst_img[1], dst_img[2]) = pixelYUVToRGB(final_y0, final_u, final_v);
      std::tie(dst_img[3], dst_img[4], dst_img[5]) = pixelYUVToRGB(final_y1, final_u, final_v);
      dst_img += 6;
    }
  }
}

// Function converts src_img from YUYV format to rgb
static void imageConvertYUYVToRGB(
  uint8_t * dst_img, uint8_t * src_img,
  int dst_start_row, int dst_end_row,
  int dst_num_cols, uint32_t stride_in_bytes)
{
  int final_y0 = 0;
  int final_u = 0;
  int final_y1 = 0;
  int final_v = 0;

  uint32_t stride_in_pixels = stride_in_bytes / 4;

  // rows in dst_img
  for (int row = dst_start_row; row < dst_end_row; row++) {
    // col iterates till num_cols / 2 since two rgb pixels processed each
    // iteration cols in dst_img
    for (int col = 0; col < dst_num_cols / 2; col++) {
      struct yuyv * src_ptr = reinterpret_cast<struct yuyv *>(src_img);
      struct yuyv * pixel = &src_ptr[col + row * stride_in_pixels];
      final_y0 = pixel->y0;
      final_u = pixel->u;
      final_y1 = pixel->y1;
      final_v = pixel->v;

      std::tie(dst_img[0], dst_img[1], dst_img[2]) = pixelYUVToRGB(final_y0, final_u, final_v);
      std::tie(dst_img[3], dst_img[4], dst_img[5]) = pixelYUVToRGB(final_y1, final_u, final_v);
      dst_img += 6;
    }
  }
}

ImageData::ImageData(
  Ogre::PixelFormat pixformat,
  const uint8_t * data_ptr,
  size_t data_size_in_bytes,
  bool take_ownership)
: pixel_format_(pixformat),
  data_ptr_(data_ptr),
  size_in_bytes_(data_size_in_bytes),
  has_ownership_(take_ownership)
{
}

ImageData::~ImageData()
{
  if (has_ownership_) {
    delete[] data_ptr_;
  }
}

void ROSImageTexture::ensureTexture(uint32_t width, uint32_t height, Ogre::PixelFormat pixel_format)
{
  if (texture_ &&
    tex_width_ == width && tex_height_ == height &&
    tex_format_ == pixel_format && tex_smooth_ == smooth_scaling_)
  {
    return;
  }

  // MIP_UNLIMITED, not MIP_DEFAULT: setNumMipmaps() takes uint32 and does no
  // translation, so MIP_DEFAULT (-1) would become 0xFFFFFFFF before backend
  // clamping.
  const uint32_t num_mips = smooth_scaling_ ? Ogre::MIP_UNLIMITED : 0;
  const int usage = smooth_scaling_ ? Ogre::TU_DEFAULT : Ogre::TU_STATIC_WRITE_ONLY;

  if (!texture_) {
    // First-time allocation. Subsequent calls reconfigure this same Ogre
    // texture in place so callers that cached the TexturePtr by name (e.g.
    // TextureUnitState::setTextureName) see the new state on the existing
    // pointer.
    texture_ = Ogre::TextureManager::getSingleton().createManual(
      texture_name_, "rviz_rendering",
      Ogre::TEX_TYPE_2D, width, height, num_mips, pixel_format, usage);
  } else {
    // freeInternalResources(), not unload(): unload() is a no-op while the
    // LoadingState is UNLOADED (manually created textures are never load()-ed),
    // so createInternalResources() below would short-circuit and silently
    // skip rebuilding the GL texture.
    texture_->freeInternalResources();
    texture_->setTextureType(Ogre::TEX_TYPE_2D);
    texture_->setWidth(width);
    texture_->setHeight(height);
    texture_->setDepth(1);
    texture_->setNumMipmaps(num_mips);
    texture_->setFormat(pixel_format);
    texture_->setUsage(usage);
    texture_->createInternalResources();
  }

  tex_width_ = width;
  tex_height_ = height;
  tex_format_ = pixel_format;
  tex_smooth_ = smooth_scaling_;
}

void ROSImageTexture::loadEmpty()
{
  ensureTexture(empty_image_.getWidth(), empty_image_.getHeight(), empty_image_.getFormat());
  texture_->getBuffer(0, 0)->blitFromMemory(empty_image_.getPixelBox());
}

template<typename T>
void
ROSImageTexture::getMinimalAndMaximalValueToNormalize(
  const T * data_ptr, size_t num_elements,
  double & min_value, double & max_value)
{
  if (normalize_) {
    // Accumulate in T to avoid per-pixel double conversions, then promote.
    T t_min = std::numeric_limits<T>::max();
    T t_max = std::numeric_limits<T>::lowest();
    const T * input_ptr = data_ptr;
    for (size_t i = 0; i < num_elements; ++i) {
      t_min = std::min(t_min, *input_ptr);
      t_max = std::max(t_max, *input_ptr);
      input_ptr++;
    }
    min_value = static_cast<double>(t_min);
    max_value = static_cast<double>(t_max);

    if (median_frames_ > 1) {
      min_value = computeMedianOfSeveralFrames(min_buffer_, min_value, this->median_frames_);
      max_value = computeMedianOfSeveralFrames(max_buffer_, max_value, this->median_frames_);
    }
  } else {
    // User-supplied min/max are doubles. Use them directly — no cast through
    // T — so a 16UC1 user can enter values up to 65535 (or above) and a
    // 32FC1 user can enter fractional values without losing precision.
    min_value = min_;
    max_value = max_;
  }
}

template<typename T>
ImageData
ROSImageTexture::convertTo8bit(const uint8_t * data_ptr, size_t data_size_in_bytes)
{
  size_t new_size_in_bytes = data_size_in_bytes / sizeof(T);

  // Zero-initialize so a degenerate range (<= 0) produces a valid all-black
  // image instead of handing uninitialized heap memory to Ogre.
  uint8_t * new_data = new uint8_t[new_size_in_bytes]();

  double min_value;
  double max_value;

  getMinimalAndMaximalValueToNormalize<T>(
    reinterpret_cast<const T *>(data_ptr), new_size_in_bytes, min_value, max_value);

  // Rescale T image and convert it to 8-bit. All arithmetic in double so
  // user-supplied min/max outside T's range (e.g. 65536 for 16UC1) still
  // produce meaningful output instead of an overflow-to-zero surprise.
  const double range = max_value - min_value;
  if (range > 0.0 && std::isfinite(range)) {
    const T * input_ptr = reinterpret_cast<const T *>(data_ptr);
    uint8_t * output_ptr = new_data;

    for (size_t i = 0; i < new_size_in_bytes; ++i, ++output_ptr, ++input_ptr) {
      double val = (static_cast<double>(*input_ptr) - min_value) / range;
      val = std::clamp(val, 0.0, 1.0);
      *output_ptr = static_cast<uint8_t>(val * 255u);
    }
  }
  // range <= 0: uniform-value frame or user-set min >= max. Leave the
  // zero-initialized buffer so we display solid black rather than garbage.

  return ImageData(Ogre::PF_BYTE_L, new_data, new_size_in_bytes, true);
}

ImageData
ROSImageTexture::convertUYVYToRGBData(const uint8_t * data_ptr, size_t data_size_in_bytes)
{
  size_t new_size_in_bytes = data_size_in_bytes * 3 / 2;

  uint8_t * new_data = new uint8_t[new_size_in_bytes];

  imageConvertUYVYToRGB(
    new_data, const_cast<uint8_t *>(data_ptr),
    0, height_, width_, stride_);

  return ImageData(Ogre::PF_BYTE_RGB, new_data, new_size_in_bytes, true);
}

ImageData
ROSImageTexture::convertYUYVToRGBData(const uint8_t * data_ptr, size_t data_size_in_bytes)
{
  size_t new_size_in_bytes = data_size_in_bytes * 3 / 2;

  uint8_t * new_data = new uint8_t[new_size_in_bytes];

  imageConvertYUYVToRGB(
    new_data, const_cast<uint8_t *>(data_ptr),
    0, height_, width_, stride_);

  return ImageData(Ogre::PF_BYTE_RGB, new_data, new_size_in_bytes, true);
}

ImageData
ROSImageTexture::convertNV12ToRGBData(const uint8_t * data_ptr, size_t data_size_in_bytes)
{
  size_t new_size_in_bytes = data_size_in_bytes * 2;

  uint8_t * new_data = new uint8_t[new_size_in_bytes];

  imageConvertNV12ToRGB(
    new_data, const_cast<uint8_t *>(data_ptr),
    0, height_, width_, stride_);

  return ImageData(Ogre::PF_BYTE_RGB, new_data, new_size_in_bytes, true);
}

ImageData
ROSImageTexture::setFormatAndNormalizeDataIfNecessary(
  const std::string & encoding, const uint8_t * data_ptr, size_t data_size_in_bytes)
{
  if (encoding == sensor_msgs::image_encodings::RGB8) {
    return ImageData(Ogre::PF_BYTE_RGB, data_ptr, data_size_in_bytes, false);
  } else if (encoding == sensor_msgs::image_encodings::RGBA8) {
    return ImageData(Ogre::PF_BYTE_RGBA, data_ptr, data_size_in_bytes, false);
  } else if (  // NOLINT enforces bracket on the same line, which makes code unreadable
    encoding == sensor_msgs::image_encodings::TYPE_8UC4 ||
    encoding == sensor_msgs::image_encodings::TYPE_8SC4 ||
    encoding == sensor_msgs::image_encodings::BGRA8)
  {
    return ImageData(Ogre::PF_BYTE_BGRA, data_ptr, data_size_in_bytes, false);
  } else if (  // NOLINT enforces bracket on the same line, which makes code unreadable
    encoding == sensor_msgs::image_encodings::TYPE_8UC3 ||
    encoding == sensor_msgs::image_encodings::TYPE_8SC3 ||
    encoding == sensor_msgs::image_encodings::BGR8)
  {
    return ImageData(Ogre::PF_BYTE_BGR, data_ptr, data_size_in_bytes, false);
  } else if (  // NOLINT enforces bracket on the same line, which makes code unreadable
    encoding == sensor_msgs::image_encodings::TYPE_8UC1 ||
    encoding == sensor_msgs::image_encodings::TYPE_8SC1 ||
    encoding == sensor_msgs::image_encodings::MONO8)
  {
    return ImageData(Ogre::PF_BYTE_L, data_ptr, data_size_in_bytes, false);
  } else if (  // NOLINT enforces bracket on the same line, which makes code unreadable
    encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
    encoding == sensor_msgs::image_encodings::TYPE_16SC1 ||
    encoding == sensor_msgs::image_encodings::MONO16)
  {
    return convertTo8bit<uint16_t>(data_ptr, data_size_in_bytes);
  } else if (encoding.find("bayer") == 0) {
    return ImageData(Ogre::PF_BYTE_L, data_ptr, data_size_in_bytes, false);
  } else if (encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    return convertTo8bit<float>(data_ptr, data_size_in_bytes);
  } else if (encoding == sensor_msgs::image_encodings::UYVY) {
    return convertUYVYToRGBData(data_ptr, data_size_in_bytes);
  } else if (encoding == sensor_msgs::image_encodings::YUYV) {
    return convertYUYVToRGBData(data_ptr, data_size_in_bytes);
  } else if (encoding == sensor_msgs::image_encodings::NV12) {
    return convertNV12ToRGBData(data_ptr, data_size_in_bytes);
  } else {
    throw UnsupportedImageEncoding(encoding);
  }
}

void ROSImageTexture::addMessage(sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  current_image_ = msg;
  new_image_ = true;
}

}  // namespace displays
}  // namespace rviz_default_plugins
