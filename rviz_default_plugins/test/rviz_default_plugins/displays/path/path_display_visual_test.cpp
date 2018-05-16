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
 *     * Neither the name of the copyright holders nor the names of its
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

#include "rviz_visual_testing_framework/visual_test_fixture.hpp"
#include "rviz_visual_testing_framework/visual_test_publisher.hpp"

#include "../../page_objects/path_display_page_object.hpp"
#include "../../publishers/path_publisher.hpp"

TEST_F(VisualTestFixture, test_with_path) {
  auto path_publisher = std::make_unique<VisualTestPublisher>(
    std::make_shared<nodes::PathPublisher>(), "path_frame");

  setCamPose(Ogre::Vector3(2, 2, 16));
  setCamLookAt(Ogre::Vector3(2, 2, 0));

  auto path_display = addDisplay<PathDisplayPageObject>();
  path_display->setTopic("/path");
  path_display->setLineStyleForPath("Billboards");
  path_display->setLineWidthForPath(20);
  path_display->setPathColor(0, 0, 200);
  path_display->setStyleOfPose("Axes");
  path_display->setPoseAxesRadius(3);
  path_display->setPoseAxesLength(8);

  assertMainWindowIdentity();
}
