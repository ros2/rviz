/*
* Copyright (c) 2017, Bosch Software Innovations GmbH.
* Copyright (c) 2018, TNG Technology Consulting GmbH.
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

#include "../../page_objects/point_cloud_common_page_object.hpp"
#include "../../publishers/illuminance_publisher.hpp"

class IlluminanceDisplayPageObject
  : public PointCloudCommonPageObject
{
public:
  IlluminanceDisplayPageObject()
  : PointCloudCommonPageObject("Illuminance")
  {}
};

TEST_F(VisualTestFixture, sphere_changes_color_depending_on_illuminance) {
  auto illuminance_publisher = std::make_shared<nodes::IlluminancePublisher>();
  auto illuminance_visual_publisher =
    std::make_unique<VisualTestPublisher>(
    illuminance_publisher, "illuminance_frame");

  setCamPose(Ogre::Vector3(0, 0, 16));
  setCamLookAt(Ogre::Vector3(0, 0, 0));

  auto illuminance_display = addDisplay<IlluminanceDisplayPageObject>();
  illuminance_display->setTopic("/illuminance");
  illuminance_display->setStyle("Spheres");
  illuminance_display->setSizeMeters(11);

  illuminance_publisher->setIlluminance(150);
  captureMainWindow("illuminance_display_low_illuminance");

  executor_->queueAction(
    [illuminance_publisher]()
    {
      illuminance_publisher->setIlluminance(800);
    });

  captureMainWindow("illuminance_display_high_illuminance");
  assertScreenShotsIdentity();
}
