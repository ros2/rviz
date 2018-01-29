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

#include "src/visual_test_fixture.hpp"
#include "src/page_objects/point_cloud_display_page_object.hpp"
#include "src/page_objects/grid_display_page_object.hpp"
#include "src/page_objects/polygon_display_page_object.hpp"


TEST_F(VisualTestFixture, example_test_structure) {
  /// Set the position of the camera and its sight vector.
  setCamPose(Ogre::Vector3(0, 3, 16));
  setCamLookAt(Ogre::Vector3(0, 2, 0));

  /// Add the desired displays and set their properties. For example:

  /// Add a PointCloud display and change points style, size and alpha:
  auto point_cloud_display = addDisplay<PointCloudDisplayPageObject>();
  point_cloud_display->setStyle("Spheres");
  point_cloud_display->setSize("1");
  point_cloud_display->setAlpha("0,5");

  /// Add a Grid display and modify its offset:
  auto grid_display = addDisplay<GridDisplayPageObject>();
  grid_display->setOffSet(1, 2, 5);

  auto second_grid_display = addDisplay<GridDisplayPageObject>();
  removeDisplay(grid_display);

  /// Take a screenshot (reference or test), and compare to the reference image (if in TEST mode).
  assertImageIdentity();
}

TEST_F(VisualTestFixture, example_test_structure2) {
  /// Set the position of the camera and its sight vector.
  setCamPose(Ogre::Vector3(0, 3, 15));
  setCamLookAt(Ogre::Vector3(0, 2, 0));

  /// Add the desired displays and set their properties. For example:

  /// Add a PointCloud display and change points style, size and alpha:
  auto grid_display = addDisplay<GridDisplayPageObject>();
  grid_display->setAlpha("0,4");

  auto grid_display2 = addDisplay<GridDisplayPageObject>();
  grid_display2->setOffSet(1.2f, 3, 4.5f);
  grid_display2->setColor(255, 0, 0);
  auto poligon_display = addDisplay<PolygonDisplayPageObject>();


  /// Take a screenshot (reference or test), and compare to the reference image (if in TEST mode).
  assertImageIdentity();
}
