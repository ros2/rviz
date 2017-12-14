/*
 * Copyright (c) 2017, Bosch Software Innovations GmbH.
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

#include <gtest/gtest.h>

#include <vector>

#include "../../../../src/rviz_default_plugins/displays/pointcloud/point_cloud2_display.hpp"
#include "./message_creators.hpp"

using namespace rviz_default_plugins::displays;  // NOLINT

TEST(PointCloud2Display, filter_keeps_valid_points) {
  // just plain Point is ambiguous on macOS
  rviz_default_plugins::Point p1 = {1, 2, 3};
  rviz_default_plugins::Point p2 = {4, 5, 6};
  auto cloud = createPointCloud2WithPoints(std::vector<rviz_default_plugins::Point>{p1, p2});

  PointCloud2Display display;
  auto filtered = display.filterOutInvalidPoints(cloud);

  ASSERT_EQ(filtered->width, (uint) 2);
  ASSERT_EQ(filtered->data.size(), (uint) sizeof(float) * 3 * 2);

  auto buffer = reinterpret_cast<const float *>(filtered->data.data());
  ASSERT_EQ(buffer[0], 1);
  ASSERT_EQ(buffer[1], 2);
  ASSERT_EQ(buffer[2], 3);
  ASSERT_EQ(buffer[3], 4);
  ASSERT_EQ(buffer[4], 5);
  ASSERT_EQ(buffer[5], 6);
}

TEST(PointCloud2Display, filter_removes_invalid_point) {
  // just plain Point is ambiguous on macOS
  rviz_default_plugins::Point p1 = {1, 2, 3};
  rviz_default_plugins::Point p2 = {1, NAN, 3};
  auto cloud = createPointCloud2WithPoints(std::vector<rviz_default_plugins::Point>{p1, p2});

  PointCloud2Display display;
  auto filtered = display.filterOutInvalidPoints(cloud);

  ASSERT_EQ(filtered->width, (uint) 1);
  ASSERT_EQ(filtered->data.size(), (uint) sizeof(float) * 3 * 1);

  auto buffer = reinterpret_cast<const float *>(filtered->data.data());
  ASSERT_EQ(buffer[0], 1);
  ASSERT_EQ(buffer[1], 2);
  ASSERT_EQ(buffer[2], 3);
}

TEST(PointCloud2Display, filter_returns_empty_cloud_if_all_points_are_invalid) {
  // just plain Point is ambiguous on macOS
  rviz_default_plugins::Point p1 = {NAN, 2, 3};
  rviz_default_plugins::Point p2 = {1, NAN, 3};
  auto cloud = createPointCloud2WithPoints(std::vector<rviz_default_plugins::Point>{p1, p2});

  PointCloud2Display display;
  auto filtered = display.filterOutInvalidPoints(cloud);

  ASSERT_EQ(filtered->width, (uint) 0);
  ASSERT_EQ(filtered->data.size(), (uint) 0);
}
