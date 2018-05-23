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

#include "ament_index_cpp/get_package_prefix.hpp"

#include "rviz_visual_testing_framework/visual_test_fixture.hpp"
#include "rviz_visual_testing_framework/transform_publisher.hpp"

#include "../../page_objects/robot_model_display_page_object.hpp"

TEST_F(VisualTestFixture, robot_model_display_test) {
  std::vector<StaticTransform> transforms;
  transforms.emplace_back(StaticTransform("map", "test_robot_link", 0, 0, 0, 0, 0, 0));
  transforms.emplace_back(StaticTransform("map", "test_robot_link_head", 0, 0, 0, 0, 0, 0));
  transforms.emplace_back(StaticTransform("map", "test_robot_link_right_arm", 0, 0, 0, 0, 0, 0));
  transforms.emplace_back(StaticTransform("map", "test_robot_link_left_arm", 0, 0, 0, 0, 0, 0));
  auto transform_publisher = std::make_unique<TransformPublisher>(transforms);

  setCamPose(Ogre::Vector3(8, 0, 0));
  setCamLookAt(Ogre::Vector3(0, 0, 0));

  auto robot_model_display = addDisplay<RobotModelDisplayPageObject>();
  robot_model_display->setDescriptionSource("File");

  QString prefix = QString::fromStdString(
    ament_index_cpp::get_package_prefix("rviz_rendering_tests"));
  robot_model_display->setFile(prefix + "/share/rviz_rendering_tests/test_meshes/test.urdf");
  robot_model_display->setVisualEnabled(true);

  captureMainWindow("robot_model_display_with_visuals");

  robot_model_display->setVisualEnabled(false);
  robot_model_display->setCollisionEnabled(true);

  captureMainWindow("robot_model_display_with_collision");

  assertScreenShotsIdentity();
}
