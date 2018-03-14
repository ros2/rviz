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

#include <vector>
#include <memory>
#include <string>

#include "../src/visual_test_fixture.hpp"
#include "../src/page_objects/camera_display_page_object.hpp"
#include "../src/page_objects/grid_display_page_object.hpp"
#include "../src/page_objects/image_display_page_object.hpp"
#include "../src/page_objects/point_cloud_display_page_object.hpp"
#include "../src/page_objects/polygon_display_page_object.hpp"

TEST_F(VisualTestFixture, example_test_structure) {
  /// Set the position of the camera and its sight vector:
  setCamPose(Ogre::Vector3(0, 3, 16));
  setCamLookAt(Ogre::Vector3(0, 2, 0));

  /// Add displays:
  auto pointcloud_display = addDisplay<PointCloudDisplayPageObject>();
  auto grid_display = addDisplay<GridDisplayPageObject>();
  auto image_display = addDisplay<ImageDisplayPageObject>();

  /// Modify their properties:
  pointcloud_display->setSize("3");
  pointcloud_display->collapse();
  grid_display->setOffset(0.3, 2, 0.4);
  grid_display->setColor(0, 255, 0);
  grid_display->collapse();
  image_display->setQueueSize("10");

  /// Take the screenshots of the desired render windows:
  captureRenderWindow(image_display);
  captureMainWindow();

  /// Compare test screenshots with the reference ones (if in TEST mode):
  assertScreenShotsIdentity();
}

TEST_F(VisualTestFixture, second_example) {
  /// Add displays:
  auto grid_display = addDisplay<GridDisplayPageObject>();

  /// Modify their properties:
  grid_display->setOffset(0.3, 2, 0.4);
  grid_display->setColor(255, 255, 0);
  grid_display->setPlaneCellCount("30");

  // If only the main window is of interest, the method 'assertMainWindowIdentity()' can instead be
  // used, which both captures the screenshot and compares it to the reference:
  assertMainWindowIdentity();
}
