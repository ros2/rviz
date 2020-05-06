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

#include <gmock/gmock.h>

#include <vector>

#include "rviz_default_plugins/displays/pointcloud/point_cloud2_display.hpp"
#include "../../pointcloud_messages.hpp"

using namespace ::testing;  // NOLINT
using namespace rviz_default_plugins::displays;  // NOLINT

TEST(PointCloud2Display, filter_keeps_valid_points) {
  // just plain Point is ambiguous on macOS
  rviz_default_plugins::Point p1 = {1, 2, 3};
  rviz_default_plugins::Point p2 = {4, 5, 6};
  auto cloud = createPointCloud2WithPoints(std::vector<rviz_default_plugins::Point>{p1, p2});

  PointCloud2Display display;
  auto filtered = display.filterOutInvalidPoints(cloud);

  ASSERT_THAT(filtered->width, Eq(2u));
  ASSERT_THAT(filtered->data, SizeIs(sizeof(float) * 3 * 2));

  auto buffer = reinterpret_cast<const float *>(filtered->data.data());
  ASSERT_THAT(buffer[0], Eq(1));
  ASSERT_THAT(buffer[1], Eq(2));
  ASSERT_THAT(buffer[2], Eq(3));
  ASSERT_THAT(buffer[3], Eq(4));
  ASSERT_THAT(buffer[4], Eq(5));
  ASSERT_THAT(buffer[5], Eq(6));
}

TEST(PointCloud2Display, hasXYZChannels_returns_true_for_valid_pointcloud) {
  // just plain Point is ambiguous on macOS
  rviz_default_plugins::Point p1 = {1, 2, 3};
  rviz_default_plugins::Point p2 = {1, NAN, 3};
  auto cloud = createPointCloud2WithPoints(std::vector<rviz_default_plugins::Point>{p1, p2});

  PointCloud2Display display;
  bool has_xyz = display.hasXYZChannels(cloud);

  ASSERT_TRUE(has_xyz);
}

TEST(PointCloud2Display, cloudDataMatchesDimensions_returns_true_for_valid_pointcloud) {
  // just plain Point is ambiguous on macOS
  rviz_default_plugins::Point p1 = {1, 2, 3};
  rviz_default_plugins::Point p2 = {1, NAN, 3};
  auto cloud = createPointCloud2WithPoints(std::vector<rviz_default_plugins::Point>{p1, p2});

  PointCloud2Display display;
  bool matches_dimensions = display.cloudDataMatchesDimensions(cloud);

  ASSERT_TRUE(matches_dimensions);
}

TEST(PointCloud2Display, filter_removes_invalid_point) {
  // just plain Point is ambiguous on macOS
  rviz_default_plugins::Point p1 = {1, 2, 3};
  rviz_default_plugins::Point p2 = {1, NAN, 3};
  auto cloud = createPointCloud2WithPoints(std::vector<rviz_default_plugins::Point>{p1, p2});

  PointCloud2Display display;
  auto filtered = display.filterOutInvalidPoints(cloud);

  ASSERT_THAT(filtered->width, Eq(1u));
  ASSERT_THAT(filtered->data, SizeIs(sizeof(float) * 3 * 1));

  auto buffer = reinterpret_cast<const float *>(filtered->data.data());
  ASSERT_THAT(buffer[0], Eq(1));
  ASSERT_THAT(buffer[1], Eq(2));
  ASSERT_THAT(buffer[2], Eq(3));
}

TEST(PointCloud2Display, filter_returns_empty_cloud_if_all_points_are_invalid) {
  // just plain Point is ambiguous on macOS
  rviz_default_plugins::Point p1 = {NAN, 2, 3};
  rviz_default_plugins::Point p2 = {1, NAN, 3};
  auto cloud = createPointCloud2WithPoints(std::vector<rviz_default_plugins::Point>{p1, p2});

  PointCloud2Display display;
  auto filtered = display.filterOutInvalidPoints(cloud);

  ASSERT_THAT(filtered->width, Eq(0u));
  ASSERT_THAT(filtered->data, IsEmpty());
}
