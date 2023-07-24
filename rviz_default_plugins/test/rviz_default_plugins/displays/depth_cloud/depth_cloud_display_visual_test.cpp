/*
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

#include <memory>
#include <string>
#include <vector>

#include "rviz_visual_testing_framework/visual_test_fixture.hpp"
#include "rviz_visual_testing_framework/visual_test_publisher.hpp"

#include "../../publishers/camera_info_publisher.hpp"
#include "../../publishers/image_publisher.hpp"
#include "../../page_objects/depth_cloud_page_object.hpp"

TEST_F(VisualTestFixture, test_depth_cloud_display_with_published_image) {
  std::vector<PublisherWithFrame> publishers = {
    PublisherWithFrame(std::make_shared<nodes::CameraInfoPublisher>(), "image"),
    PublisherWithFrame(std::make_shared<nodes::ImagePublisher>("/image/image_raw"), "image_frame"),
    PublisherWithFrame(
      std::make_shared<nodes::ImagePublisher>(
        "/rgbd_camera/image"), "depthcloud_frame")
  };
  auto cam_publisher = std::make_unique<VisualTestPublisher>(publishers);

  auto depth_cloud_display = addDisplay<DepthCloudDisplayPageObject>();
  depth_cloud_display->setTopicFilter(true);
  depth_cloud_display->setDepthMapTopic("/rgbd_camera/image");
  depth_cloud_display->setImageMapTopic("/image/image_raw");
  depth_cloud_display->setDepthMapTransport("raw");
  depth_cloud_display->setImageMapTransport("raw");
  depth_cloud_display->setQueueSize(10);
  depth_cloud_display->setOclusionCompensation(true);
  depth_cloud_display->setSelectable(false);
  depth_cloud_display->setStyle("Spheres");
  depth_cloud_display->setSize(10);
  depth_cloud_display->setAlpha(0.5);
  depth_cloud_display->setDecay(0);

  /// Compare test screenshots with the reference ones (if in TEST mode):
  assertScreenShotsIdentity();
}
