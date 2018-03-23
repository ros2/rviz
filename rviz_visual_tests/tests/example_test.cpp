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
 *     * Neither the name of the copyright holder nor the names of its contributors
 *       may be used to endorse or promote products derived from
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

#include <atomic>
#include <vector>
#include <memory>
#include <string>
#include <thread>

#include "tf2_ros/static_transform_broadcaster.h"

#include "../src/visual_test_fixture.hpp"
#include "../src/page_objects/camera_display_page_object.hpp"
#include "../src/page_objects/grid_display_page_object.hpp"
#include "../src/page_objects/image_display_page_object.hpp"
#include "../src/page_objects/point_cloud_display_page_object.hpp"
#include "../src/page_objects/polygon_display_page_object.hpp"
#include "example_nodes.hpp"

std::atomic<bool> nodes_spinning;

geometry_msgs::msg::Point32 createPoint(float x, float y, float z)
{
  geometry_msgs::msg::Point32 point;
  point.x = x;
  point.y = y;
  point.z = z;

  return point;
}

void publishPointCloud(std::vector<geometry_msgs::msg::Point32> points)
{
  auto point_cloud_publisher_node = std::make_shared<nodes::PointCloudPublisher>(points);
  auto transformer_publisher_node = std::make_shared<rclcpp::Node>("static_transform_publisher");
  tf2_ros::StaticTransformBroadcaster broadcaster(transformer_publisher_node);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(point_cloud_publisher_node);
  executor.add_node(transformer_publisher_node);

  rclcpp::WallRate loop_rate(0.2);
  auto msg = nodes::createStaticTransformMessage("map", "pointcloud_frame");
  while (nodes_spinning) {
    broadcaster.sendTransform(msg);
    executor.spin_some();
    loop_rate.sleep();
  }
}

TEST_F(VisualTestFixture, example_test_with_pointcloud) {
  nodes_spinning = true;
  std::vector<geometry_msgs::msg::Point32> points{createPoint(0, 0, 0)};
  std::thread publishers(publishPointCloud, points);

  // Set the position of the camera and its sight vector:
  setCamPose(Ogre::Vector3(0, 0, 16));
  setCamLookAt(Ogre::Vector3(0, 0, 0));

  auto pointcloud_display = addDisplay<PointCloudDisplayPageObject>();
  pointcloud_display->setStyle("Spheres");
  pointcloud_display->setSizeMeters("11");
  pointcloud_display->setColor(0, 255, 0);
  /// Take the screenshots of the desired render windows:
  captureMainWindow();

  /// Compare test screenshots with the reference ones (if in TEST mode):
  assertScreenShotsIdentity();

  nodes_spinning = false;
  publishers.join();
}
