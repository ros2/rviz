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
#include <vector>

#include "rviz_visual_testing_framework/visual_test_fixture.hpp"
#include "rviz_visual_testing_framework/visual_test_publisher.hpp"

#include "../../page_objects/map_display_page_object.hpp"
#include "../../page_objects/marker_display_page_object.hpp"
#include "../../publishers/map_publisher.hpp"
#include "../../publishers/single_marker_publisher.hpp"

TEST_F(VisualTestFixture, test_map_display_with_large_map) {
  auto publishers = std::vector<PublisherWithFrame>();
  publishers.emplace_back(PublisherWithFrame(std::make_shared<nodes::MapPublisher>(), "map_frame"));
  publishers.emplace_back(
    PublisherWithFrame(
      std::make_shared<nodes::SingleMarkerPublisher>(), "marker_frame"));
  auto map_publisher = std::make_unique<VisualTestPublisher>(publishers);

  setCamPose(Ogre::Vector3(0, 0, 200));
  setCamLookAt(Ogre::Vector3(0, 0, 0));

  auto marker_display = addDisplay<MarkerDisplayPageObject>();
  marker_display->setTopic("/marker");

  auto map_display = addDisplay<MapDisplayPageObject>();
  map_display->setTopic("/map");
  map_display->setAlpha(1);

  captureMainWindow("map_display_default_color_scheme");

  map_display->setColorScheme("raw");

  captureMainWindow("map_display_map_color_scheme");

  map_display->setColorScheme("costmap");

  captureMainWindow("map_display_costmap_scheme");

  map_display->drawUnder(true);

  captureMainWindow("map_display_rendered_below_other_objects");

  assertScreenShotsIdentity();
}
