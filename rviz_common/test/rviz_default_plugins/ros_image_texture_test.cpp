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
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
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

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <memory>
#include <vector>

#include <Ogre.h> // NOLINT

#include "sensor_msgs/image_encodings.hpp"

#include "test/rviz_rendering/ogre_testing_environment.hpp"

#include "src/rviz_default_plugins/image/ros_image_texture.hpp"

using namespace rviz_default_plugins;  // NOLINT


class RosImageTextureTestFixture : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    testing_environment_ = std::make_shared<rviz_rendering::OgreTestingEnvironment>();
    testing_environment_->setUpOgreTestEnvironment();
  }

  static std::shared_ptr<rviz_rendering::OgreTestingEnvironment> testing_environment_;
};

std::shared_ptr<rviz_rendering::OgreTestingEnvironment>
RosImageTextureTestFixture::testing_environment_ = nullptr;

TEST_F(RosImageTextureTestFixture, constructor_initializes_texture_with_default_image) {
  ROSImageTexture texture;
  Ogre::TexturePtr ogreTexture = texture.getTexture();

  Ogre::Image textureImage;
  ogreTexture->convertToImage(textureImage);

  Ogre::Image expectedImage;
  expectedImage.load("no_image.png", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

  ASSERT_THAT(
    std::vector<uint8_t>(textureImage.getData(), textureImage.getData() + textureImage.getSize()),
    testing::ElementsAreArray(expectedImage.getData(), expectedImage.getSize()));
}

TEST_F(RosImageTextureTestFixture, update_writes_new_image_to_the_texture) {
  Ogre::Image testImage;
  testImage.load("test_20x20.png", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

  sensor_msgs::msg::Image::SharedPtr msg = std::make_shared<sensor_msgs::msg::Image>();
  msg->width = testImage.getWidth();
  msg->height = testImage.getHeight();
  msg->encoding = sensor_msgs::image_encodings::RGB8;
  msg->data = std::vector<uint8_t>(
    testImage.getData(), testImage.getData() + testImage.getSize());

  ROSImageTexture texture;
  texture.addMessage(msg);
  texture.update();

  Ogre::TexturePtr ogreTexture = texture.getTexture();
  Ogre::Image textureImage;
  ogreTexture->convertToImage(textureImage);

  ASSERT_THAT(
    std::vector<uint8_t>(textureImage.getData(), textureImage.getData() + textureImage.getSize()),
    testing::ElementsAreArray(testImage.getData(), testImage.getSize()));
}

TEST_F(RosImageTextureTestFixture, normalize_adapts_values_to_range_of_0_and_255) {
  std::vector<uint16_t> uint16_vector = {1, 2, 3, 4};
  ROSImageTexture texture;
  texture.setNormalizeFloatImage(true);

  std::vector<uint8_t> buffer = texture.normalize<uint16_t>(uint16_vector.data(),
      uint16_vector.size());

  ASSERT_EQ(buffer[0], 0);
  ASSERT_EQ(buffer[1], 85);
  ASSERT_EQ(buffer[2], 170);
  ASSERT_EQ(buffer[3], 255);
}

TEST_F(RosImageTextureTestFixture, normalize_puts_all_pixels_to_0_if_equal) {
  std::vector<uint16_t> uint16_vector(100, 9);
  ROSImageTexture texture;
  texture.setNormalizeFloatImage(true);

  std::vector<uint8_t> buffer = texture.normalize<uint16_t>(uint16_vector.data(),
      uint16_vector.size());

  ASSERT_EQ(buffer.size(), static_cast<size_t>(100));
  ASSERT_EQ(buffer[0], 0);
}

TEST_F(RosImageTextureTestFixture, normalize_adapts_values_to_smaller_range_after_some_time) {
  std::vector<uint16_t> first_image = {1, 2, 3, 4};
  std::vector<uint16_t> second_image = {0, 1, 2, 3};
  std::vector<uint16_t> third_image = {2, 3};
  ROSImageTexture texture;
  texture.setMedianFrames(3);
  texture.setNormalizeFloatImage(true);

  texture.normalize<uint16_t>(first_image.data(), first_image.size());
  texture.normalize<uint16_t>(second_image.data(), second_image.size());
  std::vector<uint8_t> buffer = texture.normalize<uint16_t>(third_image.data(), third_image.size());

  // "0" is equal to the median from (0, 1, 2), "255" to the median from (3, 3, 4)
  ASSERT_EQ(buffer[0], 127);  // 2 is in the middle of lower bound (1) and upper (3)
  ASSERT_EQ(buffer[1], 255);  // 3 is upper bound as median from (3, 3, 4)
}

TEST_F(RosImageTextureTestFixture, correctly_cuts_images_if_new_pixels_are_larger_than_bound) {
  std::vector<uint16_t> first_image = {1, 2, 3, 4};
  std::vector<uint16_t> second_image = {0, 5};
  ROSImageTexture texture;
  texture.setMedianFrames(3);
  texture.setNormalizeFloatImage(true);

  texture.normalize<uint16_t>(first_image.data(), first_image.size());
  texture.normalize<uint16_t>(first_image.data(), first_image.size());
  std::vector<uint8_t> buffer = texture.normalize<uint16_t>(second_image.data(), second_image
      .size());

  ASSERT_EQ(buffer[0], 0);
  ASSERT_EQ(buffer[1], 255);
}

TEST_F(RosImageTextureTestFixture, rescale_floats_also_if_not_normalized) {
  std::vector<float> first_image = {0.333f, 0.667f};
  ROSImageTexture texture;
  texture.setNormalizeFloatImage(false);

  std::vector<uint8_t> buffer = texture.normalize<float>(first_image.data(), first_image.size());

  ASSERT_EQ(buffer[0], 84);  // default range is 0 = 0.0f, 1 = 1.0 f
  ASSERT_EQ(buffer[1], 170);
}
