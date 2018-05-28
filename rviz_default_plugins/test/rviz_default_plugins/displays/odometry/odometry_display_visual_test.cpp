/*
 * Copyright (c) 2018, Bosch Software Innovations GmbH.
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

#include <memory>
#include <string>

#include "rviz_visual_testing_framework/visual_test_fixture.hpp"
#include "rviz_visual_testing_framework/visual_test_publisher.hpp"

#include "../../page_objects/odometry_display_page_object.hpp"
#include "../../publishers/odometry_publisher.hpp"

TEST_F(VisualTestFixture, test_odometry_display) {
  auto odometry = std::make_shared<nodes::OdometryPublisher>();
  odometry->initialize();  // necessary due to missing transform_broadcaster constructor
  auto odometry_publisher = std::make_unique<VisualTestPublisher>(odometry, "odometry_frame");

  setCamPose(Ogre::Vector3(2, 2, 16));
  setCamLookAt(Ogre::Vector3(2, 2, 0));

  auto odometry_display = addDisplay<OdometryDisplayPageObject>();
  odometry_display->setTopic("/odometry");
  odometry_display->setCovarianceOrientationColorStyle("RGB");
  odometry_display->setCovarianceOrientationAlpha(1);
  odometry_display->setCovarianceOrientationScale(5);

  odometry_display->setArrowColor(100, 0, 255);
  odometry_display->setArrowAlpha(1);
  odometry_display->setArrowShaftLength(5);
  odometry_display->setArrowShaftRadius(2);
  odometry_display->setArrowHeadLength(3);
  odometry_display->setArrowHeadRadius(3);
  odometry_display->setCovariance(false);

  captureMainWindow("odometry_arrow");

  odometry_display->setShape("Axes");
  odometry_display->setAxesLength(7);
  odometry_display->setAxesRadius(3);

  captureMainWindow("odometry_axes");

  odometry_display->setShape("Arrow");
  odometry_display->setArrowAlpha(0);
  odometry_display->setCovariance(true);

  captureMainWindow("odometry_covariance_orientation");

  odometry_display->setCovarianceOrientation(false);
  odometry_display->setCovariancePositionScale(5);
  odometry_display->setCovariancePositionAlpha(1);

  captureMainWindow("odometry_covariance_position");

  assertScreenShotsIdentity();
}
