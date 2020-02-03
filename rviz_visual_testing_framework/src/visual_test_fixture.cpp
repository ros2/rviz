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

#include "rviz_visual_testing_framework/visual_test_fixture.hpp"

#include <memory>
#include <string>
#include <vector>

#include "rviz_common/ros_integration/ros_client_abstraction.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

void VisualTestFixture::SetUpTestCase()
{
  QLocale::setDefault(QLocale::English);

  int argc = 1;
  char first_argument[6] = "rviz2";
  char * argv[] = {first_argument, nullptr};

  visualizer_app_ = new rviz_common::VisualizerApp(
    std::make_unique<rviz_common::ros_integration::RosClientAbstraction>());
  qapp_ = new QApplication(argc, argv);


  visualizer_app_->setApp(qapp_);

  visualizer_app_->init(argc, argv);

  std::string package_share_directory = ament_index_cpp::get_package_share_directory(
    "rviz_visual_testing_framework");
  if (VisualTest::generateReferenceImages()) {
    visualizer_app_->loadConfig(
      QDir::toNativeSeparators(
        QString::fromStdString(
          package_share_directory +
          "/config/visual_tests_default_config.rviz")));
  } else {
    visualizer_app_->loadConfig(
      QDir::toNativeSeparators(
        QString::fromStdString(
          package_share_directory +
          "/config/visual_tests_test_image_config.rviz")));
  }
}

void VisualTestFixture::TearDown()
{
  display_handler_->removeAllDisplays();
}

void VisualTestFixture::TearDownTestCase()
{
  delete VisualTestFixture::visualizer_app_;
  delete VisualTestFixture::qapp_;
  VisualTestFixture::visualizer_app_ = nullptr;
  VisualTestFixture::qapp_ = nullptr;

  std::string reference_images_path = QDir::toNativeSeparators(
    QString::fromStdString(src_directory_path_ + "/test/reference_images/"))
    .toStdString();
  std::string test_images_path = QDir::toNativeSeparators(
    QString::fromStdString(build_directory_path_ + "/test_images/")).toStdString();

  std::cout << "[   INFO   ] The reference images are located in: " <<
    reference_images_path << "\n[   INFO   ] The test images are located in: " <<
    test_images_path << "\n";
}

void VisualTestFixture::setCamPose(Ogre::Vector3 camera_pose)
{
  visual_test_->setCamPose(camera_pose);
}

void VisualTestFixture::setCamLookAt(Ogre::Vector3 camera_look_at_vector)
{
  visual_test_->setCamLookAt(camera_look_at_vector);
}

void VisualTestFixture::updateCamWithDelay(Ogre::Vector3 new_pose, Ogre::Vector3 new_look_at)
{
  executor_->queueAction(
    [this, new_pose, new_look_at] {
      visual_test_->setCamPose(new_pose);
      visual_test_->setCamLookAt(new_look_at);
    });
}

void VisualTestFixture::setTesterThreshold(double threshold)
{
  visual_test_->setTesterThreshold(threshold);
}

void VisualTestFixture::removeDisplay(std::shared_ptr<BasePageObject> display)
{
  display_handler_->removeDisplay(display);
}

void VisualTestFixture::captureMainWindow(Ogre::String image_name)
{
  setNameIfEmpty(image_name);
  screen_shots_.push_back(image_name);
  visual_test_->takeScreenShot(image_name, nullptr);
}

void VisualTestFixture::captureRenderWindow(
  std::shared_ptr<PageObjectWithWindow> display, Ogre::String name)
{
  setNameIfEmpty(name);
  static int count = 0;
  Ogre::String image_name = name + "_secondary_window" + Ogre::StringConverter::toString(count++);
  screen_shots_.push_back(image_name);

  visual_test_->takeScreenShot(image_name, display);
}

void VisualTestFixture::assertScreenShotsIdentity()
{
  startApplication();

  for (const auto & image_name : screen_shots_) {
    visual_test_->assertVisualIdentity(image_name);
  }
}

void VisualTestFixture::assertMainWindowIdentity(Ogre::String image_name)
{
  setNameIfEmpty(image_name);
  captureMainWindow(image_name);
  startApplication();
  visual_test_->assertVisualIdentity(image_name);
}

void VisualTestFixture::startApplication()
{
  executor_->queueAction([] {qapp_->quit();});
  qapp_->exec();
}

void VisualTestFixture::setNameIfEmpty(Ogre::String & name)
{
  if (name.empty()) {
    name = test_name_;
  }
}

void VisualTestFixture::wait(size_t milliseconds_to_wait)
{
  executor_->wait(milliseconds_to_wait);
}

QApplication * VisualTestFixture::qapp_ = nullptr;
rviz_common::VisualizerApp * VisualTestFixture::visualizer_app_ = nullptr;
std::string VisualTestFixture::src_directory_path_;  // NOLINT
std::string VisualTestFixture::build_directory_path_;  // NOLINT
