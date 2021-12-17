/*
 * Copyright (c) 2018, Bosch Software Innovations GmbH.
 * Copyright (c) 2020, Sarthak Mittal.
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

#include "../../page_objects/particle_cloud_display_page_object.hpp"
#include "../../publishers/particle_cloud_publisher.hpp"

TEST_F(VisualTestFixture, particle_cloud_visual_test) {
  auto path_publisher = std::make_unique<VisualTestPublisher>(
    std::make_shared<nodes::ParticleCloudPublisher>(), "particle_cloud_frame");

  setCamPose(Ogre::Vector3(2.7f, 0, 8));

  auto particle_cloud_display = addDisplay<ParticleCloudDisplayPageObject>();
  particle_cloud_display->setTopic("/particle_cloud");
  particle_cloud_display->setShape("Arrow (3D)");
  particle_cloud_display->setColor(254, 182, 6);

  particle_cloud_display->setMinArrowLength(3);
  particle_cloud_display->setMaxArrowLength(3);
  captureMainWindow("particle_cloud_display_with_arrow3d");

  particle_cloud_display->setAlpha(0);
  captureMainWindow("empty_scene");

  particle_cloud_display->setShape("Axes");
  updateCamWithDelay(Ogre::Vector3(7, 7, 7), Ogre::Vector3(0, 0, 0));
  particle_cloud_display->setMinArrowLength(6);
  particle_cloud_display->setMaxArrowLength(6);
  captureMainWindow("particle_cloud_display_with_axes");

  assertScreenShotsIdentity();
}
