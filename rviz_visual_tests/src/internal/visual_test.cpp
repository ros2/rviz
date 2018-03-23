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
#include "visual_test.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <string>

#include <QDir>  // NOLINT

VisualTest::VisualTest(rviz_common::VisualizerApp * vapp, std::shared_ptr<Executor> executor)
: default_cam_pose_(Ogre::Vector3(0, 0, 15)),
  default_cam_look_at_(Ogre::Vector3(0, 0, 0)),
  scene_(vapp, default_cam_pose_, default_cam_look_at_, executor),
  tester_("", ""),
  build_directory_path_(_BUILD_DIR_PATH),
  source_directory_path_(_SRC_DIR_PATH)
{
  reference_images_path_suffix_ = "/tests/reference_images/";
  test_images_path_suffix_ = "/test_images/";

  std::string reference_images_path = QDir::toNativeSeparators(
    QString::fromStdString(source_directory_path_ + reference_images_path_suffix_)).toStdString();
  std::string test_images_path = QDir::toNativeSeparators(
    QString::fromStdString(build_directory_path_ + test_images_path_suffix_)).toStdString();

  tester_ = ImageTester(reference_images_path, test_images_path);
}

VisualTest::~VisualTest()
{
  reset();
}

void VisualTest::setCamPose(Ogre::Vector3 camera_pose)
{
  scene_.setCamPose(camera_pose);
}

void VisualTest::setCamLookAt(Ogre::Vector3 look_at_vector)
{
  scene_.setLookAt(look_at_vector);
}

void VisualTest::takeReferenceScreenShot(
  Ogre::String screenshot_name, std::shared_ptr<PageObjectWithWindow> display)
{
  std::string images_name = QDir::toNativeSeparators(
    QString::fromStdString(source_directory_path_ + reference_images_path_suffix_ +
    screenshot_name)).toStdString();

  if (display) {
    display->captureDisplayRenderWindow(images_name + "_ref.png");
    return;
  }
  scene_.takeReferenceShot(images_name);
}

void VisualTest::takeTestScreenShot(
  Ogre::String screenshot_name, std::shared_ptr<PageObjectWithWindow> display)
{
  std::string images_name = QDir::toNativeSeparators(
    QString::fromStdString(build_directory_path_ + test_images_path_suffix_ +
    screenshot_name)).toStdString();

  if (display) {
    display->captureDisplayRenderWindow(images_name + ".png");
    return;
  }
  scene_.takeTestShot(images_name);
}

void VisualTest::assertVisualIdentity(Ogre::String name)
{
  if (!_REF_IMAGES && checkImageExists(name)) {
    tester_.compare(name);
  } else if (!_REF_IMAGES && !checkImageExists(name)) {
    GTEST_FAIL() << "\n[  ERROR   ] Reference image does not exist, or its name is incorrect (it "
      "should be: '" << name << "_ref.png'.)\n";
  } else {
    std::cout << "\n[   INFO:  ] The reference image '" << name << "_ref.png' has been updated "
      "correctly.\n\n";
    SUCCEED();
  }
}

void VisualTest::takeScreenShot(Ogre::String name, std::shared_ptr<PageObjectWithWindow> display)
{
  if (directoriesDoNotExist()) {
    GTEST_FAIL() << "[  ERROR   ] at least one of test_images and reference_images directories "
      "doesn't exist. Make sure that both directory are correctly placed and try again. \n";
  }

  if (_REF_IMAGES) {
    takeReferenceScreenShot(name, display);
  } else {
    takeTestScreenShot(name, display);
  }
}

void VisualTest::setCamera()
{
  scene_.setUpCamera();
  scene_.installCamera();
}

bool VisualTest::checkImageExists(std::string & name)
{
  const std::string source_path = _SRC_DIR_PATH;
  const std::string reference_image_name = QDir::toNativeSeparators(
    QString::fromStdString(source_path + reference_images_path_suffix_ +
    name + "_ref.png")).toStdString();
  struct stat buffer;

  return stat(reference_image_name.c_str(), &buffer) == 0;
}

bool VisualTest::directoriesDoNotExist()
{
  const std::string source_path = _SRC_DIR_PATH;
  const std::string build_path = _BUILD_DIR_PATH;
  const std::string reference_directory = QDir::toNativeSeparators(QString::fromStdString(
        source_path + reference_images_path_suffix_)).toStdString();
  const std::string test_directory = QDir::toNativeSeparators(QString::fromStdString(build_path +
      test_images_path_suffix_)).toStdString();
  struct stat buffer;

  bool test_images_directory_exists = stat(test_directory.c_str(), &buffer) == 0;
  bool reference_images_directory_exists = stat(reference_directory.c_str(), &buffer) == 0;

  return !(test_images_directory_exists && reference_images_directory_exists);
}

void VisualTest::reset()
{
  Ogre::MeshManager::getSingleton().removeAll();
  setCamPose(default_cam_pose_);
  setCamLookAt(default_cam_look_at_);
}
