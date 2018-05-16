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

#ifndef RVIZ_VISUAL_TESTING_FRAMEWORK__INTERNAL__IMAGE_TESTER_HPP_
#define RVIZ_VISUAL_TESTING_FRAMEWORK__INTERNAL__IMAGE_TESTER_HPP_

#ifdef _WIN32
# pragma warning(push)
# pragma warning(disable : 4996)
#endif
#include <Ogre.h>
#ifdef _WIN32
# pragma warning(pop)
#endif

class ImageTester
{
public:
  ImageTester(Ogre::String reference_directory_path, Ogre::String test_directory_path);
  void compare(Ogre::String image_name);
  void computeImageDifference(
    Ogre::Image test_image, Ogre::Image reference_image, Ogre::String image_name);

  double computeMseIndex(Ogre::String image_name, size_t image_width, size_t image_height);
  void setThreshold(double threshold);

private:
  void resizeAndCropImage(
    const Ogre::String & image_name, Ogre::Image & test_image,
    size_t reference_image_width, size_t reference_image_height, size_t test_image_width,
    size_t test_image_height);
  void cropImageWidthToFitReference(Ogre::Image & test_image, size_t reference_image_width);
  void resizeImageKeepingProportions(
    Ogre::Image & test_image, size_t reference_image_height,
    size_t test_image_width, size_t test_image_height) const;
  Ogre::Image cropImage(const Ogre::Image & source, size_t offset, size_t cropped_width);
  Ogre::Image loadImage(Ogre::String image_name, Ogre::String image_directory_path);
  size_t pixelDifference(
    Ogre::Image reference_image, Ogre::Image test_image, size_t width, size_t height);
  void assertImageIdentity(
    Ogre::String image_name, Ogre::Image reference_image, Ogre::Image test_image);

  Ogre::String reference_directory_path_;
  Ogre::String test_directory_path_;
  double threshold_;
};
#endif  // RVIZ_VISUAL_TESTING_FRAMEWORK__INTERNAL__IMAGE_TESTER_HPP_
