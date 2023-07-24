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

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <exception>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <rclcpp/rclcpp.hpp>

#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"
#include "rviz_common/visibility_control.hpp"

// This is necessary because of using stl types here. It is completely safe, because
// a) the member is not accessible from the outside
// b) there are no inline functions.
#ifdef _WIN32
# pragma warning(push)
# pragma warning(disable:4251)
#endif

namespace rviz_common
{
class MultiLayerDepthException : public std::exception
{
public:
  RVIZ_COMMON_PUBLIC
  explicit MultiLayerDepthException(const std::string & error_msg)
  : std::exception(), error_msg_(error_msg)
  {
  }
  ~MultiLayerDepthException() throw() override
  {
  }

  RVIZ_COMMON_PUBLIC
  const char * what() const throw() override
  {
    return error_msg_.c_str();
  }

protected:
  std::string error_msg_;
};

class RVIZ_COMMON_PUBLIC MultiLayerDepth final
{
public:
  MultiLayerDepth()
  : shadow_time_out_(30.0), shadow_distance_(0.01f) {}

  virtual ~MultiLayerDepth()
  {
  }

  void setShadowTimeOut(double time_out);

  void enableOcclusionCompensation(bool occlusion_compensation);

  sensor_msgs::msg::PointCloud2::SharedPtr
  generatePointCloudFromDepth(
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & color_msg,
    sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_msg,
    ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node);

  void reset();

private:
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
    std::vector<uint32_t> & rgba_color_raw,
    ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node);

  // Helpers to generate pointcloud2 message
  sensor_msgs::msg::PointCloud2::SharedPtr initPointCloud();
  void finalizePointCloud(
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

#ifdef _WIN32
# pragma warning(pop)
#endif

#endif  // RVIZ_COMMON__DEPTH_CLOUD_MLD_HPP_
