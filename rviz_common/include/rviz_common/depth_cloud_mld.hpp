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

#ifndef RVIZ_COMMON__DEPTH_CLOUD_MLD_HPP_
#define RVIZ_COMMON__DEPTH_CLOUD_MLD_HPP_

#include <cstdint>
#include <cmath>
#include <exception>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <rclcpp/rclcpp.hpp>

namespace rviz_common
{
class MultiLayerDepthException : public std::exception
{
public:
  explicit MultiLayerDepthException(const std::string & error_msg)
  : std::exception(), error_msg_(error_msg)
  {
  }
  ~MultiLayerDepthException() throw() override
  {
  }

  const char * what() const throw() override
  {
    return error_msg_.c_str();
  }

protected:
  std::string error_msg_;
};

class MultiLayerDepth
{
public:
  MultiLayerDepth()
  : shadow_time_out_(30.0), shadow_distance_(0.01) {}
  virtual ~MultiLayerDepth()
  {
  }

  void setShadowTimeOut(double time_out)
  {
    shadow_time_out_ = time_out;
  }

  void enableOcclusionCompensation(bool occlusion_compensation)
  {
    occlusion_compensation_ = occlusion_compensation;
    reset();
  }

  sensor_msgs::msg::PointCloud2::SharedPtr
  generatePointCloudFromDepth(
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & color_msg,
    sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_msg);
  void reset()
  {
    if (occlusion_compensation_) {
      // reset shadow buffer
      memset(&shadow_depth_[0], 0, sizeof(float) * shadow_depth_.size());
      memset(&shadow_buffer_[0], 0, sizeof(uint8_t) * shadow_buffer_.size());
      memset(&shadow_timestamp_[0], 0, sizeof(double) * shadow_timestamp_.size());
    }
  }

protected:
  /** @brief Precompute projection matrix, initialize buffers */
  void initializeConversion(
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
    sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg);

  /** @brief Convert color data to RGBA format */
  template<typename T>
  void convertColor(
    const sensor_msgs::msg::Image::ConstSharedPtr & color_msg,
    std::vector<uint32_t> & rgba_color_raw);

  /** @brief Generate single-layered depth cloud (depth only) */
  template<typename T>
  sensor_msgs::msg::PointCloud2::SharedPtr generatePointCloudSL(
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
    std::vector<uint32_t> & rgba_color_raw);

  /** @brief Generate multi-layered depth cloud (depth+shadow) */
  template<typename T>
  sensor_msgs::msg::PointCloud2::SharedPtr generatePointCloudML(
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
    std::vector<uint32_t> & rgba_color_raw);

  // Helpers to generate pointcloud2 message
  sensor_msgs::msg::PointCloud2::SharedPtr initPointCloud();
  void finalizingPointCloud(
    sensor_msgs::msg::PointCloud2::SharedPtr & point_cloud,
    std::size_t size);

  std::vector<float> projection_map_x_;
  std::vector<float> projection_map_y_;

  // shadow buffers
  std::vector<float> shadow_depth_;
  std::vector<double> shadow_timestamp_;
  std::vector<uint8_t> shadow_buffer_;

  // configuration
  bool occlusion_compensation_;
  double shadow_time_out_;
  float shadow_distance_;
};
}  // namespace rviz_common
#endif  // RVIZ_COMMON__DEPTH_CLOUD_MLD_HPP_
