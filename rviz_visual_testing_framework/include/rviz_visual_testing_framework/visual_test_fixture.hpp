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

#ifndef RVIZ_VISUAL_TESTING_FRAMEWORK__VISUAL_TEST_FIXTURE_HPP_
#define RVIZ_VISUAL_TESTING_FRAMEWORK__VISUAL_TEST_FIXTURE_HPP_

#include <memory>
#include <vector>

#include "rviz_visual_testing_framework/internal/display_handler.hpp"
#include "rviz_visual_testing_framework/internal/executor.hpp"
#include "rviz_visual_testing_framework/internal/visual_test.hpp"
#include "rviz_visual_testing_framework/page_objects/page_object_with_window.hpp"

class VisualTestFixture : public testing::Test
{
public:
  VisualTestFixture();
  static void SetUpTestCase();
  void TearDown() override;
  static void TearDownTestCase();

  void setCamPose(Ogre::Vector3 camera_pose);
  void setCamLookAt(Ogre::Vector3 camera_look_at_vector);
  template<typename T>
  std::shared_ptr<T> addDisplay()
  {
    return display_handler_->addDisplay<T>();
  }
  void removeDisplay(std::shared_ptr<BasePageObject> display);

  void captureMainWindow(Ogre::String image_name = "");
  void captureRenderWindow(
    std::shared_ptr<PageObjectWithWindow> display, Ogre::String name = "");

  void assertScreenShotsIdentity();
  void assertMainWindowIdentity(Ogre::String image_name = "");

  Ogre::String test_name_;
  std::unique_ptr<VisualTest> visual_test_;
  std::unique_ptr<DisplayHandler> display_handler_;
  std::shared_ptr<std::vector<int>> all_display_ids_vector_;
  std::vector<Ogre::String> screen_shots_;
  std::shared_ptr<Executor> executor_;
  static QApplication * qapp_;
  static rviz_common::VisualizerApp * visualizer_app_;

private:
  void startApplication();
  void setNameIfEmpty(Ogre::String & name);
};

#endif  // RVIZ_VISUAL_TESTING_FRAMEWORK__VISUAL_TEST_FIXTURE_HPP_
