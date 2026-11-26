// Copyright (c) 2026, Open Source Robotics Foundation, Inc.
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


#include <gmock/gmock.h>

#include <cstdint>
#include <memory>
#include <string>

#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "rviz_common/depth_cloud_mld.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"

using namespace ::testing;  // NOLINT

namespace
{

sensor_msgs::msg::CameraInfo::SharedPtr makeCameraInfo(uint32_t width, uint32_t height)
{
  auto info = std::make_shared<sensor_msgs::msg::CameraInfo>();
  info->width = width;
  info->height = height;
  info->p[0] = 500.0;
  info->p[2] = width / 2.0;
  info->p[5] = 500.0;
  info->p[6] = height / 2.0;
  info->p[10] = 1.0;
  return info;
}

sensor_msgs::msg::Image::SharedPtr makeImage(
  const std::string & encoding, uint32_t width, uint32_t height, uint32_t bytes_per_pixel,
  size_t data_size)
{
  auto image = std::make_shared<sensor_msgs::msg::Image>();
  image->encoding = encoding;
  image->width = width;
  image->height = height;
  image->step = width * bytes_per_pixel;
  image->data.resize(data_size);
  return image;
}

}  // namespace

class DepthCloudMldTest : public Test
{
protected:
  void SetUp() override
  {
    mld_.enableOcclusionCompensation(false);
  }

  rviz_common::MultiLayerDepth mld_;
  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr no_node_;
};

TEST_F(DepthCloudMldTest, well_formed_depth_image_is_converted) {
  auto camera_info = makeCameraInfo(8, 8);
  auto depth = makeImage("32FC1", 8, 8, 4, 8 * 8 * 4);

  sensor_msgs::msg::PointCloud2::SharedPtr cloud;
  ASSERT_NO_THROW(
    cloud = mld_.generatePointCloudFromDepth(depth, nullptr, camera_info, no_node_));
  ASSERT_THAT(cloud, NotNull());
}

TEST_F(DepthCloudMldTest, undersized_depth_image_is_rejected) {
  auto camera_info = makeCameraInfo(64, 64);
  auto depth = makeImage("32FC1", 64, 64, 4, 16);

  EXPECT_THROW(
    mld_.generatePointCloudFromDepth(depth, nullptr, camera_info, no_node_),
    rviz_common::MultiLayerDepthException);
}

TEST_F(DepthCloudMldTest, depth_image_with_overflowing_byte_size_is_rejected) {
  auto camera_info = makeCameraInfo(1u << 31, 1u << 31);
  auto depth = makeImage("32FC1", 1u << 31, 1u << 31, 4, 16);

  EXPECT_THROW(
    mld_.generatePointCloudFromDepth(depth, nullptr, camera_info, no_node_),
    rviz_common::MultiLayerDepthException);
}

TEST_F(DepthCloudMldTest, undersized_color_image_is_rejected) {
  auto camera_info = makeCameraInfo(64, 64);
  auto depth = makeImage("32FC1", 64, 64, 4, 64 * 64 * 4);
  auto color = makeImage("rgb8", 64, 64, 3, 12);

  EXPECT_THROW(
    mld_.generatePointCloudFromDepth(depth, color, camera_info, no_node_),
    rviz_common::MultiLayerDepthException);
}
