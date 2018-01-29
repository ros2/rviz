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
#ifndef INTERNAL__VISUAL_TEST_HPP_
#define INTERNAL__VISUAL_TEST_HPP_

#include <string>

#ifdef _WIN32
# pragma warning(push)
# pragma warning(disable : 4996)
#endif
#include <Ogre.h>
#ifdef _WIN32
# pragma warning(pop)
#endif

#include "rviz_scene_test.hpp"
#include "image_tester.hpp"

class VisualTest
{
public:
  /// Initializes the scene and sets the path to the image directories.
  VisualTest(
    QApplication * qapp,
    rviz_common::VisualizerApp * vapp);

  ///  Sets the position of the camera.
  void setCamPose(Ogre::Vector3 camera_pose);

  /// Sets the look direction of the camera.
  void setCamLookAt(Ogre::Vector3 look_at_vector);

  /** If the command line option VisualTests is set to TEST, it performs a pixel-by-pixel
      comparison of the test image with name.png and the reference image name_ref.png. It will
      fail if either the reference image does not exist or if the two images are not identical.
  */
  void assertVisualIdentity(Ogre::String name);

  /** Depending on the value of the command line option VisualTests, it either takes a screenshot
      and saves it into [...]/rviz_ws/build/test_framework/reference_images as name_ref.png, or
      into [...]/rviz_ws/build/test_framework/test_images as name.png.
  */
  void takeScreenShot(Ogre::String name, uint64_t delay);

  /** Reset the scene to the default settings. i.e. repositions the camera, its lookAt vector and
      deletes all the renderables (a part the grid).
  */
  void reset();

  void setCamera();

private:
  void takeReferenceScreenShot(Ogre::String screenshot_name, uint64_t delay);
  void takeTestScreenShot(Ogre::String screenshot_name, uint64_t delay);
  bool checkImageExists(std::string & name);
  bool directoriesDoNotExist();

  Ogre::Vector3 default_cam_pose_;
  Ogre::Vector3 default_cam_look_at_;
  RvizTestScene scene_;
  ImageTester tester_;
  std::string test_images_path_suffix_;
  std::string reference_images_path_suffix_;
  std::string build_directory_path_;
  std::string source_directory_path_;
};

#endif  // INTERNAL__VISUAL_TEST_HPP_
