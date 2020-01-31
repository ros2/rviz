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
#include "../../publishers/temperature_publisher.hpp"

class TemperatureDisplayPageObject
  : public PointCloudCommonPageObject
{
public:
  TemperatureDisplayPageObject()
  : PointCloudCommonPageObject("Temperature")
  {}
};

TEST_F(VisualTestFixture, sphere_changes_color_depending_on_temperature) {
  auto temperature_publisher = std::make_shared<nodes::TemperaturePublisher>();
  auto temperature_visual_publisher =
    std::make_unique<VisualTestPublisher>(
    temperature_publisher, "temperature_frame");

  setCamPose(Ogre::Vector3(0, 0, 16));
  setCamLookAt(Ogre::Vector3(0, 0, 0));

  auto temperature_display = addDisplay<TemperatureDisplayPageObject>();
  temperature_display->setTopic("/temperature");
  temperature_display->setStyle("Spheres");
  temperature_display->setSizeMeters(11);

  temperature_publisher->setTemperature(15.);
  captureMainWindow("temperature_display_low_temperature");

  executor_->queueAction(
    [temperature_publisher]()
    {
      temperature_publisher->setTemperature(85.);
    });

  captureMainWindow("temperature_display_high_temperature");
  assertScreenShotsIdentity();
}
