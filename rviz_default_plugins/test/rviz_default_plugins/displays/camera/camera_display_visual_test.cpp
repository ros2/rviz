/*
 * Copyright (c) 2018, Bosch Software Innovations GmbH.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
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

#include "../../page_objects/camera_display_page_object.hpp"
#include "../../publishers/camera_info_publisher.hpp"
#include "../../publishers/image_publisher.hpp"
#include "../../page_objects/point_cloud_common_page_object.hpp"
#include "../../publishers/point_cloud_publisher.hpp"

class PointCloudDisplayPageObject
  : public PointCloudCommonPageObject
{
public:
  PointCloudDisplayPageObject()
  : PointCloudCommonPageObject("PointCloud")
  {}
};

TEST_F(VisualTestFixture, test_camera_display_with_published_image) {
  auto points = {nodes::createPoint(0, 0, 10)};
  std::vector<PublisherWithFrame> publishers = {
    PublisherWithFrame(std::make_shared<nodes::CameraInfoPublisher>(), "image"),
    PublisherWithFrame(std::make_shared<nodes::ImagePublisher>(), "image_frame"),
    PublisherWithFrame(std::make_shared<nodes::PointCloudPublisher>(points), "pointcloud_frame")
  };
  auto cam_publisher = std::make_unique<VisualTestPublisher>(publishers);

  setCamPose(Ogre::Vector3(0, 0, 16));
  setCamLookAt(Ogre::Vector3(0, 0, 0));

  auto camera_display = addDisplay<CameraDisplayPageObject>();
  camera_display->setTopic("/image");
  camera_display->collapse();

  auto pointcloud_display = addDisplay<PointCloudDisplayPageObject>();
  pointcloud_display->setTopic("/pointcloud");
  pointcloud_display->setStyle("Spheres");
  pointcloud_display->setSizeMeters(11);
  pointcloud_display->setColor(0, 0, 255);

  captureRenderWindow(camera_display);

  assertScreenShotsIdentity();
}
