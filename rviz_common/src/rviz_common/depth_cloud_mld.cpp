/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * Copyright (c) 2023, Open Source Robotics Foundation, Inc.
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
#include "rviz_common/depth_cloud_mld.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <sstream>
#include <vector>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"

constexpr size_t POINT_STEP = (sizeof(float) * 4);

namespace rviz_common
{
// Encapsulate differences between processing float and uint16_t depths
template<typename T>
struct DepthTraits
{
};

template<>
struct DepthTraits<uint16_t>
{
  static inline bool valid(float depth)
  {
    return depth != 0.0;
  }
  static inline float toMeters(uint16_t depth)
  {
    return depth * 0.001f;
  }
};

template<>
struct DepthTraits<float>
{
  static inline bool valid(float depth)
  {
    return std::isfinite(depth);
  }
  static inline float toMeters(float depth)
  {
    return depth;
  }
};


struct RGBA
{
  uint8_t red;
  uint8_t green;
  uint8_t blue;
  uint8_t alpha;
};

void MultiLayerDepth::setShadowTimeOut(double time_out)
{
  this->shadow_time_out_ = time_out;
}

void MultiLayerDepth::enableOcclusionCompensation(bool occlusion_compensation)
{
  occlusion_compensation_ = occlusion_compensation;
  reset();
}

void MultiLayerDepth::reset()
{
  if (occlusion_compensation_) {
    // reset shadow buffer
    memset(&shadow_depth_[0], 0, sizeof(float) * shadow_depth_.size());
    memset(&shadow_buffer_[0], 0, sizeof(uint8_t) * shadow_buffer_.size());
    memset(&shadow_timestamp_[0], 0, sizeof(double) * shadow_timestamp_.size());
  }
}


void MultiLayerDepth::initializeConversion(
  const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
  sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg)
{
  if (!depth_msg || !camera_info_msg) {
    std::string error_msg("Waiting for CameraInfo message..");
    throw(rviz_common::MultiLayerDepthException(error_msg));
  }

  // do some sanity checks
  uint32_t binning_x = camera_info_msg->binning_x > 1 ? camera_info_msg->binning_x : 1;
  uint32_t binning_y = camera_info_msg->binning_y > 1 ? camera_info_msg->binning_y : 1;

  uint32_t roi_width =
    camera_info_msg->roi.width > 0 ? camera_info_msg->roi.width : camera_info_msg->width;
  uint32_t roi_height =
    camera_info_msg->roi.height > 0 ? camera_info_msg->roi.height : camera_info_msg->height;

  uint32_t expected_width = roi_width / binning_x;
  uint32_t expected_height = roi_height / binning_y;

  if (expected_width != depth_msg->width || expected_height != depth_msg->height) {
    std::ostringstream s;
    s << "Depth image size and camera info don't match: ";
    s << depth_msg->width << " x " << depth_msg->height;
    s << " vs " << expected_width << " x " << expected_height;
    s << "(binning: " << binning_x << " x " << binning_y;
    s << ", ROI size: " << roi_width << " x " << roi_height << ")";
    throw(rviz_common::MultiLayerDepthException(s.str()));
  }

  uint32_t width = depth_msg->width;
  uint32_t height = depth_msg->height;

  uint32_t size = width * height;

  if (size != static_cast<uint32_t>(shadow_depth_.size())) {
    // Allocate memory for shadow processing
    shadow_depth_.resize(size, 0.0f);
    shadow_timestamp_.resize(size, 0.0);
    shadow_buffer_.resize(size * POINT_STEP, 0);

    // Precompute 3D projection matrix
    //
    // The following computation of center_x,y and fx,fy duplicates
    // code in the image_geometry package, but this avoids a dependency
    // on OpenCV, which simplifies releasing rviz.

    // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
    double scale_x = camera_info_msg->binning_x > 1 ? (1.0 / camera_info_msg->binning_x) : 1.0;
    double scale_y = camera_info_msg->binning_y > 1 ? (1.0 / camera_info_msg->binning_y) : 1.0;

    // Use correct principal point from calibration
    float center_x =
      static_cast<float>((camera_info_msg->p[2] - camera_info_msg->roi.x_offset) * scale_x);
    float center_y =
      static_cast<float>((camera_info_msg->p[6] - camera_info_msg->roi.y_offset) * scale_y);

    double fx = camera_info_msg->p[0] * scale_x;
    double fy = camera_info_msg->p[5] * scale_y;

    float constant_x = static_cast<float>(1.0f / fx);
    float constant_y = static_cast<float>(1.0f / fy);

    projection_map_x_.resize(width);
    projection_map_y_.resize(height);
    std::vector<float>::iterator projX = projection_map_x_.begin();
    std::vector<float>::iterator projY = projection_map_y_.begin();

    // precompute 3D projection matrix
    for (uint32_t v = 0; v < height; ++v, ++projY) {
      *projY = (v - center_y) * constant_y;
    }

    for (uint32_t u = 0; u < width; ++u, ++projX) {
      *projX = (u - center_x) * constant_x;
    }

    // reset shadow vectors
    reset();
  }
}


template<typename T>
sensor_msgs::msg::PointCloud2::SharedPtr
MultiLayerDepth::generatePointCloudSL(
  const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
  std::vector<uint32_t> & rgba_color_raw)
{
  sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg = initPointCloud();
  cloud_msg->data.resize(depth_msg->height * depth_msg->width * cloud_msg->point_step);

  uint32_t * color_img_ptr = nullptr;

  if (!rgba_color_raw.empty()) {
    color_img_ptr = &rgba_color_raw[0];
  }

  ////////////////////////////////////////////////
  // depth map to point cloud conversion
  ////////////////////////////////////////////////

  float * cloud_data_ptr = reinterpret_cast<float *>(&cloud_msg->data[0]);

  std::size_t point_count = 0;
  std::size_t point_idx = 0;

  const T * depth_img_ptr = reinterpret_cast<const T *>(&depth_msg->data[0]);

  std::vector<float>::iterator proj_x;
  std::vector<float>::const_iterator proj_x_end = projection_map_x_.end();

  std::vector<float>::iterator proj_y;
  std::vector<float>::const_iterator proj_y_end = projection_map_y_.end();

  // iterate over projection matrix
  for (proj_y = projection_map_y_.begin(); proj_y != proj_y_end; ++proj_y) {
    for (proj_x = projection_map_x_.begin(); proj_x != proj_x_end;
      ++proj_x, ++point_idx, ++depth_img_ptr)
    {
      T depth_raw = *depth_img_ptr;
      if (DepthTraits<T>::valid(depth_raw)) {
        float depth = DepthTraits<T>::toMeters(depth_raw);

        // define point color
        uint32_t color;
        if (color_img_ptr) {
          color = *color_img_ptr;
        } else {
          color = ((uint32_t)255 << 16 | (uint32_t)255 << 8 | (uint32_t)255);
        }

        // fill in X,Y,Z and color
        *cloud_data_ptr = (*proj_x) * depth;
        ++cloud_data_ptr;
        *cloud_data_ptr = (*proj_y) * depth;
        ++cloud_data_ptr;
        *cloud_data_ptr = depth;
        ++cloud_data_ptr;
        float ret;
        std::memcpy(&ret, &color, sizeof(float));
        *cloud_data_ptr = ret;
        ++cloud_data_ptr;

        ++point_count;
      }

      // increase color iterator pointer
      if (color_img_ptr) {
        ++color_img_ptr;
      }
    }
  }

  finalizePointCloud(cloud_msg, point_count);

  return cloud_msg;
}


template<typename T>
sensor_msgs::msg::PointCloud2::SharedPtr
MultiLayerDepth::generatePointCloudML(
  const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
  std::vector<uint32_t> & rgba_color_raw,
  ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node)
{
  sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg = initPointCloud();
  cloud_msg->data.resize(depth_msg->height * depth_msg->width * cloud_msg->point_step * 2);

  uint32_t * color_img_ptr = nullptr;

  if (!rgba_color_raw.empty()) {
    color_img_ptr = &rgba_color_raw[0];
  }

  ////////////////////////////////////////////////
  // depth map to point cloud conversion
  ////////////////////////////////////////////////

  float * cloud_data_ptr = reinterpret_cast<float *>(&cloud_msg->data[0]);
  uint8_t * cloud_shadow_buffer_ptr = &shadow_buffer_[0];

  const std::size_t point_step = cloud_msg->point_step;

  std::size_t point_count = 0;
  std::size_t point_idx = 0;

  double time_now = rviz_ros_node.lock()->get_raw_node()->now().seconds();
  double time_expire = time_now - shadow_time_out_;

  const T * depth_img_ptr = reinterpret_cast<const T *>(&depth_msg->data[0]);

  std::vector<float>::iterator proj_x;
  std::vector<float>::const_iterator proj_x_end = projection_map_x_.end();

  std::vector<float>::iterator proj_y;
  std::vector<float>::const_iterator proj_y_end = projection_map_y_.end();

  // iterate over projection matrix
  for (proj_y = projection_map_y_.begin(); proj_y != proj_y_end; ++proj_y) {
    for (proj_x = projection_map_x_.begin(); proj_x != proj_x_end;
      ++proj_x, ++point_idx, ++depth_img_ptr, cloud_shadow_buffer_ptr += point_step)
    {
      // lookup shadow depth
      float shadow_depth = shadow_depth_[point_idx];

      // check for time-outs
      if ((shadow_depth != 0.0f) && (shadow_timestamp_[point_idx] < time_expire)) {
        // clear shadow pixel
        shadow_depth = shadow_depth_[point_idx] = 0.0f;
      }

      T depth_raw = *depth_img_ptr;
      if (DepthTraits<T>::valid(depth_raw)) {
        float depth = DepthTraits<T>::toMeters(depth_raw);

        // pointer to current point data
        float * cloud_data_pixel_ptr = cloud_data_ptr;

        // define point color
        uint32_t color;
        if (color_img_ptr) {
          color = *color_img_ptr;
        } else {
          color = ((uint32_t)255 << 16 | (uint32_t)255 << 8 | (uint32_t)255);
        }

        // fill in X,Y,Z and color
        *cloud_data_ptr = (*proj_x) * depth;
        ++cloud_data_ptr;
        *cloud_data_ptr = (*proj_y) * depth;
        ++cloud_data_ptr;
        *cloud_data_ptr = depth;
        ++cloud_data_ptr;
        float ret;
        std::memcpy(&ret, &color, sizeof(float));
        *cloud_data_ptr = ret;
        ++cloud_data_ptr;

        ++point_count;

        // if shadow point exists -> display it
        if (depth < shadow_depth - shadow_distance_) {
          // copy point data from shadow buffer to point cloud
          memcpy(cloud_data_ptr, cloud_shadow_buffer_ptr, point_step);
          cloud_data_ptr += 4;
          ++point_count;
        } else {
          // save a copy of current point to shadow buffer
          memcpy(cloud_shadow_buffer_ptr, cloud_data_pixel_ptr, point_step);

          // reduce color intensity in shadow buffer
          RGBA * color = reinterpret_cast<RGBA *>(cloud_shadow_buffer_ptr + sizeof(float) * 3);
          color->red /= 2;
          color->green /= 2;
          color->blue /= 2;

          // update shadow depth & time out
          shadow_depth_[point_idx] = depth;
          shadow_timestamp_[point_idx] = time_now;
        }
      } else {
        // current depth pixel is invalid -> check shadow buffer
        if (shadow_depth != 0) {
          // copy shadow point to point cloud
          memcpy(cloud_data_ptr, cloud_shadow_buffer_ptr, point_step);
          cloud_data_ptr += 4;
          ++point_count;
        }
      }

      // increase color iterator pointer
      if (color_img_ptr) {
        ++color_img_ptr;
      }
    }
  }

  finalizePointCloud(cloud_msg, point_count);

  return cloud_msg;
}


template<typename T>
void MultiLayerDepth::convertColor(
  const sensor_msgs::msg::Image::ConstSharedPtr & color_msg,
  std::vector<uint32_t> & rgba_color_raw)
{
  if (color_msg->encoding.find("rgb") == std::string::npos &&
    color_msg->encoding.find("bgr") == std::string::npos)
  {
    throw rviz_common::MultiLayerDepthException("Encoded type not supported!");
  }

  size_t i;
  size_t num_pixel = color_msg->width * color_msg->height;

  // query image properties
  int num_channels = sensor_msgs::image_encodings::numChannels(color_msg->encoding);

  bool rgb_encoding = false;
  if (color_msg->encoding.find("rgb") != std::string::npos) {
    rgb_encoding = true;
  }

  bool has_alpha = sensor_msgs::image_encodings::hasAlpha(color_msg->encoding);

  // prepare output vector
  rgba_color_raw.clear();
  rgba_color_raw.reserve(num_pixel);

  // pointer to most significant byte
  const uint8_t * img_ptr = reinterpret_cast<const uint8_t *>(&color_msg->data[sizeof(T) - 1]);

  // color conversion
  switch (num_channels) {
    case 1:
      // grayscale image
      for (i = 0; i < num_pixel; ++i) {
        uint8_t gray_value = *img_ptr;
        img_ptr += sizeof(T);

        rgba_color_raw.push_back(
          (uint32_t)gray_value << 16 | (uint32_t)gray_value << 8 |
            (uint32_t)gray_value);
      }
      break;
    case 3:
    case 4:
      // rgb/bgr encoding
      for (i = 0; i < num_pixel; ++i) {
        uint8_t color1 = *(reinterpret_cast<const uint8_t *>(img_ptr));
        img_ptr += sizeof(T);
        uint8_t color2 = *(reinterpret_cast<const uint8_t *>(img_ptr));
        img_ptr += sizeof(T);
        uint8_t color3 = *(reinterpret_cast<const uint8_t *>(img_ptr));
        img_ptr += sizeof(T);

        if (has_alpha) {
          img_ptr += sizeof(T);  // skip alpha values
        }
        if (rgb_encoding) {
          // rgb encoding
          rgba_color_raw.push_back(
            (uint32_t)color1 << 16 | (uint32_t)color2 << 8 | (uint32_t)color3 << 0);
        } else {
          // bgr encoding
          rgba_color_raw.push_back(
            (uint32_t)color3 << 16 | (uint32_t)color2 << 8 | (uint32_t)color1 << 0);
        }
      }
      break;
    default:
      break;
  }
}

sensor_msgs::msg::PointCloud2::SharedPtr
MultiLayerDepth::generatePointCloudFromDepth(
  const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr & color_msg,
  sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_msg,
  ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node)
{
  if (!camera_info_msg) {
    throw rviz_common::MultiLayerDepthException("Camera info missing!");
  }

  // Add data to multi depth image
  sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_out;

  // Bit depth of image encoding
  int bitDepth = sensor_msgs::image_encodings::bitDepth(depth_msg->encoding);
  int numChannels = sensor_msgs::image_encodings::numChannels(depth_msg->encoding);

  // precompute projection matrix and initialize shadow buffer
  initializeConversion(depth_msg, camera_info_msg);

  std::vector<uint32_t> rgba_color_raw_;

  if (color_msg) {
    if (depth_msg->width != color_msg->width || depth_msg->height != color_msg->height) {
      std::stringstream error_msg;
      error_msg << "Depth image resolution (" << static_cast<int>(depth_msg->width) << "x" <<
        static_cast<int>(depth_msg->height)
                << ") "
        "does not match color image resolution ("
                << static_cast<int>(color_msg->width) << "x"
                << static_cast<int>(color_msg->height) << ")";
      throw(rviz_common::MultiLayerDepthException(error_msg.str()));
    }

    // convert color coding to 8-bit rgb data
    switch (sensor_msgs::image_encodings::bitDepth(color_msg->encoding)) {
      case 8:
        convertColor<uint8_t>(color_msg, rgba_color_raw_);
        break;
      case 16:
        convertColor<uint16_t>(color_msg, rgba_color_raw_);
        break;
      default:
        std::string error_msg("Color image has invalid bit depth");
        throw(rviz_common::MultiLayerDepthException(error_msg));
        break;
    }
  }

  if (!occlusion_compensation_) {
    // generate single layer depth cloud

    if ((bitDepth == 32) && (numChannels == 1)) {
      // floating point encoded depth map
      point_cloud_out = generatePointCloudSL<float>(depth_msg, rgba_color_raw_);
    } else if ((bitDepth == 16) && (numChannels == 1)) {
      // 32bit integer encoded depth map
      point_cloud_out = generatePointCloudSL<uint16_t>(depth_msg, rgba_color_raw_);
    }
  } else {
    // generate two layered depth cloud (depth+shadow)

    if ((bitDepth == 32) && (numChannels == 1)) {
      // floating point encoded depth map
      point_cloud_out = generatePointCloudML<float>(depth_msg, rgba_color_raw_, rviz_ros_node);
    } else if ((bitDepth == 16) && (numChannels == 1)) {
      // 32bit integer encoded depth map
      point_cloud_out = generatePointCloudML<uint16_t>(depth_msg, rgba_color_raw_, rviz_ros_node);
    }
  }

  if (!point_cloud_out) {
    std::string error_msg("Depth image has invalid format (only 16 bit and float are supported)!");
    throw(rviz_common::MultiLayerDepthException(error_msg));
  }

  return point_cloud_out;
}

sensor_msgs::msg::PointCloud2::SharedPtr MultiLayerDepth::initPointCloud()
{
  sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_out =
    std::make_shared<sensor_msgs::msg::PointCloud2>();

  point_cloud_out->fields.resize(4);
  std::size_t point_offset = 0;

  point_cloud_out->fields[0].name = "x";
  point_cloud_out->fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  point_cloud_out->fields[0].count = 1;
  point_cloud_out->fields[0].offset = static_cast<uint32_t>(point_offset);
  point_offset += sizeof(float);

  point_cloud_out->fields[1].name = "y";
  point_cloud_out->fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  point_cloud_out->fields[1].count = 1;
  point_cloud_out->fields[1].offset = static_cast<uint32_t>(point_offset);
  point_offset += sizeof(float);

  point_cloud_out->fields[2].name = "z";
  point_cloud_out->fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  point_cloud_out->fields[2].count = 1;
  point_cloud_out->fields[2].offset = static_cast<uint32_t>(point_offset);
  point_offset += sizeof(float);

  point_cloud_out->fields[3].name = "rgb";
  point_cloud_out->fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
  point_cloud_out->fields[3].count = 1;
  point_cloud_out->fields[3].offset = static_cast<uint32_t>(point_offset);
  point_offset += sizeof(float);

  point_cloud_out->point_step = static_cast<uint32_t>(point_offset);

  point_cloud_out->is_bigendian = false;
  point_cloud_out->is_dense = false;

  return point_cloud_out;
}

void MultiLayerDepth::finalizePointCloud(
  sensor_msgs::msg::PointCloud2::SharedPtr & point_cloud,
  std::size_t size)
{
  point_cloud->width = static_cast<uint32_t>(size);
  point_cloud->height = static_cast<uint32_t>(1);
  point_cloud->data.resize(point_cloud->height * point_cloud->width * point_cloud->point_step);
  point_cloud->row_step = point_cloud->point_step * point_cloud->width;
}

}  // namespace rviz_common
