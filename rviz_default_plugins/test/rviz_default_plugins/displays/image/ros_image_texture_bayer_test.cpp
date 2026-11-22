// Copyright (c) 2026, Arne Baeyens.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#include <gmock/gmock.h>

#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include <OgreImage.h>  // NOLINT
#include <OgreTexture.h>  // NOLINT

#include "sensor_msgs/image_encodings.hpp"

#include "../../ogre_testing_environment.hpp"

#include "rviz_default_plugins/displays/image/ros_image_texture.hpp"

using namespace rviz_default_plugins::displays;  // NOLINT


class RosImageTextureBayerTestFixture : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    // Ogre is a singleton, and parameterised subclasses each get
    // their own SetUpTestCase call, guard against re-initialising it.
    if (!testing_environment_) {
      testing_environment_ = std::make_shared<rviz_default_plugins::OgreTestingEnvironment>();
      testing_environment_->setUpOgreTestEnvironment();
    }
  }

  static std::shared_ptr<rviz_default_plugins::OgreTestingEnvironment> testing_environment_;
};

std::shared_ptr<rviz_default_plugins::OgreTestingEnvironment>
RosImageTextureBayerTestFixture::testing_environment_ = nullptr;


// The mosaic layout determines which sensel color sits at which 2x2 position:
//   RGGB -> (0,0)=R, (0,1)=G, (1,0)=G, (1,1)=B
//   BGGR -> (0,0)=B, (0,1)=G, (1,0)=G, (1,1)=R
//   GBRG -> (0,0)=G, (0,1)=B, (1,0)=R, (1,1)=G
//   GRBG -> (0,0)=G, (0,1)=R, (1,0)=B, (1,1)=G
// Tests build a layout-specific mosaic representing a known scene and verify
// that the demosaiced output matches the expectation. Per-layout construction
// is required: a mosaic that "means red" under one layout means a different
// color under another, so a layout-swap bug surfaces as wrong output colors.

namespace
{

struct LayoutOffsets
{
  int r_y;
  int r_x;
  int b_y;
  int b_x;
};

LayoutOffsets offsetsFor(const std::string & encoding)
{
  namespace enc = sensor_msgs::image_encodings;
  if (encoding == enc::BAYER_RGGB8 || encoding == enc::BAYER_RGGB16) {return {0, 0, 1, 1};}
  if (encoding == enc::BAYER_BGGR8 || encoding == enc::BAYER_BGGR16) {return {1, 1, 0, 0};}
  if (encoding == enc::BAYER_GBRG8 || encoding == enc::BAYER_GBRG16) {return {1, 0, 0, 1};}
  if (encoding == enc::BAYER_GRBG8 || encoding == enc::BAYER_GRBG16) {return {0, 1, 1, 0};}
  ADD_FAILURE() << "Test bug: offsetsFor() called with unknown encoding: " << encoding;
  return {0, 0, 1, 1};
}

// Build a mosaic of a uniform scene with given (R, G, B) channel intensities.
// T is the sensel storage type.
template<typename T>
std::vector<uint8_t> buildUniformMosaic(
  uint32_t width, uint32_t height,
  T r_val, T g_val, T b_val,
  const std::string & encoding)
{
  const LayoutOffsets off = offsetsFor(encoding);
  std::vector<uint8_t> data(static_cast<size_t>(width) * height * sizeof(T));
  for (uint32_t y = 0; y < height; ++y) {
    for (uint32_t x = 0; x < width; ++x) {
      const int yp = static_cast<int>(y & 1u);
      const int xp = static_cast<int>(x & 1u);
      T v;
      if (yp == off.r_y && xp == off.r_x) {
        v = r_val;
      } else if (yp == off.b_y && xp == off.b_x) {
        v = b_val;
      } else {
        v = g_val;
      }
      const size_t idx = (static_cast<size_t>(y) * width + x) * sizeof(T);
      std::memcpy(data.data() + idx, &v, sizeof(T));
    }
  }
  return data;
}

sensor_msgs::msg::Image::SharedPtr makeImage(
  uint32_t width, uint32_t height,
  const std::string & encoding,
  std::vector<uint8_t> data,
  uint32_t bytes_per_pixel)
{
  auto msg = std::make_shared<sensor_msgs::msg::Image>();
  msg->width = width;
  msg->height = height;
  msg->encoding = encoding;
  msg->step = width * bytes_per_pixel;
  msg->data = std::move(data);
  return msg;
}

// Read the texture content back as a flat RGB byte vector (size = h * w * 3).
// Ogre may store the texture internally as RGBA depending on backend/format;
// strip the alpha channel here so callers can index uniformly
// with 3 bytes per pixel.
std::vector<uint8_t> readTextureRGB(ROSImageTexture & texture)
{
  Ogre::TexturePtr ogre_texture = texture.getTexture();
  Ogre::Image image;
  ogre_texture->convertToImage(image);
  const uint8_t * data = image.getData();
  const size_t total = image.getSize();
  const uint32_t w = image.getWidth();
  const uint32_t h = image.getHeight();
  const size_t pixels = static_cast<size_t>(w) * h;
  if (pixels == 0) {return {};}
  const size_t bpp = total / pixels;

  std::vector<uint8_t> out(pixels * 3);
  for (size_t i = 0; i < pixels; ++i) {
    out[i * 3 + 0] = data[i * bpp + 0];
    out[i * 3 + 1] = data[i * bpp + 1];
    out[i * 3 + 2] = data[i * bpp + 2];
  }
  return out;
}

// Expect the texture to be width x height pixels, all of the given colour.
void expectUniformColour(
  ROSImageTexture & texture, uint32_t width, uint32_t height,
  uint8_t r, uint8_t g, uint8_t b)
{
  const std::vector<uint8_t> rgb = readTextureRGB(texture);
  ASSERT_EQ(rgb.size(), static_cast<size_t>(width) * height * 3);
  for (uint32_t y = 0; y < height; ++y) {
    for (uint32_t x = 0; x < width; ++x) {
      const size_t i = (static_cast<size_t>(y) * width + x) * 3;
      EXPECT_EQ(rgb[i + 0], r) << "R at (" << y << "," << x << ")";
      EXPECT_EQ(rgb[i + 1], g) << "G at (" << y << "," << x << ")";
      EXPECT_EQ(rgb[i + 2], b) << "B at (" << y << "," << x << ")";
    }
  }
}
}  // namespace

// A pure-red scene must come out pure red, for every layout and bit depth.
// (The position of R within the 2x2 cell identifies the layout, so with a
//  wrong layout the R samples end up in the G or B output channel.)
class RosImageTextureBayerLayoutTestFixture
  : public RosImageTextureBayerTestFixture,
  public ::testing::WithParamInterface<std::string>
{};

TEST_P(RosImageTextureBayerLayoutTestFixture, pure_red_scene_is_demosaiced_to_pure_red) {
  const std::string & encoding = GetParam();
  const uint32_t w = 8;
  const uint32_t h = 8;
  const bool is_16bit = sensor_msgs::image_encodings::bitDepth(encoding) == 16;

  // For 16-bit, the default auto-normalisation maps our sample range
  // of [0, 1023] onto [0, 255].
  auto data = is_16bit ?
    buildUniformMosaic<uint16_t>(w, h, 1023, 0, 0, encoding) :
    buildUniformMosaic<uint8_t>(w, h, 200, 0, 0, encoding);
  const uint8_t expected_r = is_16bit ? 255 : 200;

  ROSImageTexture texture;
  texture.addMessage(makeImage(w, h, encoding, std::move(data), is_16bit ? 2 : 1));
  ASSERT_TRUE(texture.update());
  expectUniformColour(texture, w, h, expected_r, 0, 0);
}

INSTANTIATE_TEST_SUITE_P(
  AllEncodings,
  RosImageTextureBayerLayoutTestFixture,
  ::testing::Values(
    sensor_msgs::image_encodings::BAYER_RGGB8,
    sensor_msgs::image_encodings::BAYER_BGGR8,
    sensor_msgs::image_encodings::BAYER_GBRG8,
    sensor_msgs::image_encodings::BAYER_GRBG8,
    sensor_msgs::image_encodings::BAYER_RGGB16,
    sensor_msgs::image_encodings::BAYER_BGGR16,
    sensor_msgs::image_encodings::BAYER_GBRG16,
    sensor_msgs::image_encodings::BAYER_GRBG16));

// With a fixed range, 16-bit values should be rescaled linearly from [Min, Max]
// onto [0, 255]: (700 - 100) / (1100 - 100) * 255 = 153.
TEST_F(RosImageTextureBayerTestFixture, 16bit_fixed_range_rescales_linearly) {
  const std::string encoding = sensor_msgs::image_encodings::BAYER_RGGB16;
  const uint32_t w = 8;
  const uint32_t h = 8;

  ROSImageTexture texture;
  texture.setNormalizeFloatImage(false, 100.0, 1100.0);
  texture.addMessage(makeImage(
    w, h, encoding, buildUniformMosaic<uint16_t>(w, h, 700, 700, 700, encoding), 2));
  ASSERT_TRUE(texture.update());
  expectUniformColour(texture, w, h, 153, 153, 153);
}

// Corner case: 2x2 image
TEST_F(RosImageTextureBayerTestFixture, small_2x2_image_is_demosaiced) {
  const std::string encoding = sensor_msgs::image_encodings::BAYER_RGGB8;
  ROSImageTexture texture;
  texture.addMessage(makeImage(
    2, 2, encoding, buildUniformMosaic<uint8_t>(2, 2, 200, 0, 0, encoding), 1));
  ASSERT_TRUE(texture.update());
  expectUniformColour(texture, 2, 2, 200, 0, 0);
}

// Reject unknown bayer encodings
TEST_F(RosImageTextureBayerTestFixture, unknown_encoding_throws_via_update) {
  auto msg = makeImage(4, 4, "bayer_xyzzy8", std::vector<uint8_t>(16, 0u), 1);

  ROSImageTexture texture;
  texture.addMessage(msg);

  EXPECT_THROW(texture.update(), UnsupportedImageEncoding);
}
