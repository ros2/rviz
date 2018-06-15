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

#include <gmock/gmock.h>

#include <memory>
#include <vector>

#ifdef _WIN32
# pragma warning(push)
# pragma warning(disable : 4996)
#endif

#include <Ogre.h>  // NOLINT

#ifdef _WIN32
# pragma warning(pop)
#endif

#include "sensor_msgs/image_encodings.hpp"

#include "test/rviz_rendering/ogre_testing_environment.hpp"

#include "include/rviz_default_plugins/displays/image/ros_image_texture.hpp"

using namespace rviz_default_plugins::displays;  // NOLINT


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
