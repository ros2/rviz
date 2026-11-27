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
#include <cstring>
#include <memory>
#include <set>
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

TEST_F(DepthCloudMldTest, rejected_depth_image_does_not_affect_next_image) {
  auto rejected_camera_info = makeCameraInfo(65536, 65537);
  auto rejected_depth = makeImage("32FC1", 65536, 65537, 4, 16);

  EXPECT_THROW(
    mld_.generatePointCloudFromDepth(rejected_depth, nullptr, rejected_camera_info, no_node_),
    rviz_common::MultiLayerDepthException);

  auto camera_info = makeCameraInfo(256, 256);
  auto depth = makeImage("32FC1", 256, 256, 4, 256 * 256 * 4);

  sensor_msgs::msg::PointCloud2::SharedPtr cloud;
  ASSERT_NO_THROW(
    cloud = mld_.generatePointCloudFromDepth(depth, nullptr, camera_info, no_node_));
  ASSERT_THAT(cloud, NotNull());
  EXPECT_THAT(cloud->width, Eq(256u * 256u));
}

TEST_F(DepthCloudMldTest, reshaped_depth_image_uses_its_own_dimensions) {
  auto first_camera_info = makeCameraInfo(4, 2);
  auto first_depth = makeImage("32FC1", 4, 2, 4, 4 * 2 * 4);
  ASSERT_NO_THROW(
    mld_.generatePointCloudFromDepth(first_depth, nullptr, first_camera_info, no_node_));

  auto camera_info = makeCameraInfo(2, 4);
  auto depth = makeImage("32FC1", 2, 4, 4, 2 * 4 * 4);
  const float one = 1.0f;
  for (size_t i = 0; i < depth->data.size(); i += sizeof(float)) {
    std::memcpy(&depth->data[i], &one, sizeof(float));
  }

  sensor_msgs::msg::PointCloud2::SharedPtr cloud;
  ASSERT_NO_THROW(
    cloud = mld_.generatePointCloudFromDepth(depth, nullptr, camera_info, no_node_));
  ASSERT_THAT(cloud, NotNull());
  ASSERT_THAT(cloud->width, Eq(2u * 4u));

  std::set<float> xs;
  std::set<float> ys;
  for (uint32_t i = 0; i < cloud->width; ++i) {
    float xy[2];
    std::memcpy(xy, &cloud->data[i * cloud->point_step], sizeof(xy));
    xs.insert(xy[0]);
    ys.insert(xy[1]);
  }
  EXPECT_THAT(xs, SizeIs(2));
  EXPECT_THAT(ys, SizeIs(4));
}

TEST_F(DepthCloudMldTest, undersized_color_image_is_rejected) {
  auto camera_info = makeCameraInfo(64, 64);
  auto depth = makeImage("32FC1", 64, 64, 4, 64 * 64 * 4);
  auto color = makeImage("rgb8", 64, 64, 3, 12);

  EXPECT_THROW(
    mld_.generatePointCloudFromDepth(depth, color, camera_info, no_node_),
    rviz_common::MultiLayerDepthException);
}
