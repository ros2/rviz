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

#include "rviz_visual_testing_framework/internal/image_tester.hpp"

#include <gtest/gtest.h>

ImageTester::ImageTester(Ogre::String reference_directory_path, Ogre::String test_directory_path)
: reference_directory_path_(reference_directory_path),
  test_directory_path_(test_directory_path),
  threshold_(0.01)
{}

void ImageTester::compare(Ogre::String image_name)
{
  Ogre::String reference_image_name = image_name + "_ref.png";
  Ogre::String test_image_name = image_name + ".png";

  Ogre::Image reference_image = loadImage(reference_image_name, reference_directory_path_);
  Ogre::Image test_image = loadImage(test_image_name, test_directory_path_);

  assertImageIdentity(image_name, reference_image, test_image);
}

Ogre::Image ImageTester::loadImage(Ogre::String image_name, Ogre::String image_directory_path)
{
  std::ifstream image_file(
    Ogre::String(image_directory_path + image_name).c_str(), std::ios::in | std::ios::binary);

  Ogre::DataStreamPtr image_data = Ogre::DataStreamPtr(
    OGRE_NEW Ogre::FileStreamDataStream(&image_file, false));

  Ogre::Image image = Ogre::Image();
  image.load(image_data, "png");

  return image;
}

void ImageTester::assertImageIdentity(
  Ogre::String image_name, Ogre::Image reference_image, Ogre::Image test_image)
{
  size_t reference_image_width = reference_image.getWidth();
  size_t reference_image_height = reference_image.getHeight();
  size_t test_image_width = test_image.getWidth();
  size_t test_image_height = test_image.getHeight();

  if (reference_image_height * test_image_width < test_image_height * reference_image_width) {
    GTEST_FAIL() << "[  ERROR   ] The test image '" << image_name + ".png" << "' is different "
      "from the reference one. The test image is not wide enough and cannot be cropped correctly" <<
      ".\n";
  }

  resizeAndCropImage(
    image_name, test_image, reference_image_width,
    reference_image_height, test_image_width, test_image_height);

  size_t different_pixels_number = pixelDifference(
    reference_image, test_image, reference_image_width, reference_image_height);

  if (different_pixels_number == 0) {
    SUCCEED();
  } else {
    computeImageDifference(test_image, reference_image, image_name);
    double mse_index = computeMseIndex(image_name, reference_image_width, reference_image_height);
    if (mse_index <= threshold_) {
      std::cout << "[   INFO   ] The test image '" << image_name + ".png" << "' is not "
        "pixel-wise identical to its reference, but the MSE index is " << mse_index <<
        ", which is not bigger than the set threshold of " << threshold_ << ".\n";
      SUCCEED();
    } else {
      GTEST_FAIL() << "[  ERROR   ] The test image '" << image_name + ".png" << "' is different "
        "from the reference one. The image difference has been computed and saved in test_images"
        ". The MSE index is equal to " << mse_index << ", which is bigger than the set threshold "
        "of " << threshold_ << ".\n";
    }
  }
}

void ImageTester::resizeAndCropImage(
  const Ogre::String & image_name,
  Ogre::Image & test_image,
  size_t reference_image_width,
  size_t reference_image_height,
  size_t test_image_width,
  size_t test_image_height)
{
  resizeImageKeepingProportions(
    test_image, reference_image_height, test_image_width, test_image_height);

  cropImageWidthToFitReference(test_image, reference_image_width);

  test_image.save(test_directory_path_ + image_name + ".png");
}

void ImageTester::resizeImageKeepingProportions(
  Ogre::Image & test_image,
  size_t reference_image_height,
  size_t test_image_width,
  size_t test_image_height) const
{
  if (reference_image_height != test_image_height) {
    auto same_ratio_test_width = static_cast<uint64_t>(
      floor(1.0 * test_image_width * reference_image_height / test_image_height));

    test_image.resize(
      static_cast<uint16_t>(same_ratio_test_width), static_cast<uint16_t>(reference_image_height));
  }
}

void ImageTester::cropImageWidthToFitReference(
  Ogre::Image & test_image, size_t reference_image_width)
{
  if (test_image.getWidth() != reference_image_width) {
    // The test config is wider than the reference config s.th. the test image is always wider
    assert(test_image.getWidth() >= reference_image_width);

    auto crop_left = static_cast<uint64_t>(
      floor(1.0 * (test_image.getWidth() - reference_image_width) / 2));
    test_image = cropImage(test_image, crop_left, reference_image_width);
  }
}

Ogre::Image ImageTester::cropImage(
  const Ogre::Image & source, size_t offset, size_t cropped_width)
{
  Ogre::Image cropped_image = Ogre::Image(source);
  cropped_image.resize(
    static_cast<uint16_t>(cropped_width), static_cast<uint16_t>(source.getHeight()));

  for (size_t row = 0; row < source.getHeight(); row++) {
    for (size_t col = 0; col < cropped_width; col++) {
      Ogre::ColourValue color = source.getColourAt(offset + col, row, 0);
      cropped_image.setColourAt(color, col, row, 0);
    }
  }

  return cropped_image;
}

size_t ImageTester::pixelDifference(
  Ogre::Image reference_image, Ogre::Image test_image,
  size_t width, size_t height)
{
  size_t different_pixels_number = 0;

  for (size_t i = 0; i < width; i++) {
    for (size_t j = 0; j < height; j++) {
      Ogre::ColourValue reference_color = reference_image.getColourAt(i, j, 0);
      Ogre::ColourValue test_color = test_image.getColourAt(i, j, 0);
      if (reference_color != test_color) {
        different_pixels_number++;
      }
    }
  }

  return different_pixels_number;
}

void ImageTester::computeImageDifference(
  Ogre::Image test_image, Ogre::Image reference_image, Ogre::String image_name)
{
  Ogre::Image image_difference = test_image;

  for (size_t i = 0; i < test_image.getWidth(); ++i) {
    for (size_t j = 0; j < test_image.getHeight(); ++j) {
      Ogre::ColourValue reference_color = reference_image.getColourAt(i, j, 0);
      Ogre::ColourValue test_color = test_image.getColourAt(i, j, 0);

      Ogre::ColourValue squared_difference_color =
        (test_color - reference_color) * (test_color - reference_color);
      Ogre::ColourValue absolute_value_difference_color = Ogre::ColourValue(
        Ogre::Math::Sqrt(squared_difference_color.r),
        Ogre::Math::Sqrt(squared_difference_color.g),
        Ogre::Math::Sqrt(squared_difference_color.b),
        Ogre::Math::Sqrt(squared_difference_color.a)
      );
      image_difference.setColourAt(absolute_value_difference_color, i, j, 0);
    }
  }

  image_difference.save(test_directory_path_ + image_name + "_diff.png");
}

double ImageTester::computeMseIndex(
  Ogre::String image_name, size_t image_width, size_t image_height)
{
  Ogre::Image diff_image = loadImage(image_name + "_diff.png", test_directory_path_);
  auto averaged_colour = Ogre::ColourValue(0, 0, 0, 0);

  for (size_t i = 0; i < image_width; ++i) {
    for (size_t j = 0; j < image_height; ++j) {
      auto colour = diff_image.getColourAt(i, j, 0);
      averaged_colour += (colour * colour);
    }
  }
  averaged_colour /= static_cast<float>(image_width * image_height);

  return (averaged_colour.r + averaged_colour.g + averaged_colour.b) / 3.f;
}


void ImageTester::setThreshold(double threshold)
{
  threshold_ = threshold;
}
