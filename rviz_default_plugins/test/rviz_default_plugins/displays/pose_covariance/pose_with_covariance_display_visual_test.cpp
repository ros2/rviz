/*
 * Copyright (c) 2018, Bosch Software Innovations GmbH.
 * Copyright (c) 2019, Martin Idel
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

// Adapted from odometry display visual test

#include <memory>
#include <string>

#include "rviz_visual_testing_framework/visual_test_fixture.hpp"
#include "rviz_visual_testing_framework/visual_test_publisher.hpp"

#include "../../page_objects/pose_with_covariance_display_page_object.hpp"
#include "../../publishers/pose_with_covariance_publisher.hpp"

TEST_F(VisualTestFixture, test_pose_with_covariance_display) {
  auto pose_with_covariance_publisher = std::make_unique<VisualTestPublisher>(
    std::make_shared<nodes::PoseWithCovariancePublisher>(),
    "pose_with_covariance_frame");

  setCamPose(Ogre::Vector3(2, 2, 16));
  setCamLookAt(Ogre::Vector3(2, 2, 0));

  auto pose_with_covariance_display = addDisplay<PoseWithCovarianceDisplayPageObject>();
  pose_with_covariance_display->setTopic("/pose_with_covariance");
  pose_with_covariance_display->setCovarianceOrientationColorStyle("RGB");
  pose_with_covariance_display->setCovarianceOrientationAlpha(1);
  pose_with_covariance_display->setCovarianceOrientationScale(5);

  pose_with_covariance_display->setArrowColor(100, 0, 255);
  pose_with_covariance_display->setArrowShaftLength(5);
  pose_with_covariance_display->setArrowShaftRadius(2);
  pose_with_covariance_display->setArrowHeadLength(3);
  pose_with_covariance_display->setArrowHeadRadius(3);
  pose_with_covariance_display->setCovariance(false);

  captureMainWindow("pose_with_covariance_arrow");

  pose_with_covariance_display->setArrowAlpha(0);
  pose_with_covariance_display->setCovariance(true);

  captureMainWindow("pose_with_covariance_orientation");

  pose_with_covariance_display->setCovarianceOrientation(false);
  pose_with_covariance_display->setCovariancePositionScale(5);
  pose_with_covariance_display->setCovariancePositionAlpha(1);

  captureMainWindow("pose_with_covariance_position");

  assertScreenShotsIdentity();
}
