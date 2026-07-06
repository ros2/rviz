// Copyright (c) 2017, Bosch Software Innovations GmbH.
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

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include <Ogre.h>  // NOLINT
#include <OgreTextureManager.h>  // NOLINT

#include "sensor_msgs/image_encodings.hpp"

#include "../../ogre_testing_environment.hpp"

#include "rviz_default_plugins/displays/image/ros_image_texture.hpp"

using namespace rviz_default_plugins::displays;  // NOLINT


class RosImageTextureTestFixture : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    // Idempotent: Ogre is a singleton, and parameterised subclasses each get
    // their own SetUpTestCase call. Guard against re-initialising it.
    if (!testing_environment_) {
      testing_environment_ = std::make_shared<rviz_default_plugins::OgreTestingEnvironment>();
      testing_environment_->setUpOgreTestEnvironment();
    }
  }

  static std::shared_ptr<rviz_default_plugins::OgreTestingEnvironment> testing_environment_;
};

std::shared_ptr<rviz_default_plugins::OgreTestingEnvironment>
RosImageTextureTestFixture::testing_environment_ = nullptr;

TEST_F(RosImageTextureTestFixture, constructor_initializes_texture_with_default_image) {
  ROSImageTexture texture;
  Ogre::TexturePtr ogreTexture = texture.getTexture();

  Ogre::Image textureImage;
  ogreTexture->convertToImage(textureImage);

  Ogre::Image expectedImage;
  expectedImage.load("no_image.png", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

#if OGRE_MIN_VERSION(13, 4, 3)
  ASSERT_THAT(
    std::vector<uint8_t>(textureImage.getData(), textureImage.getData() + textureImage.getSize()),
    testing::ElementsAreArray(expectedImage.getData(), expectedImage.getSize()));
#else
  // Can't compare the two images directly because the of a bug that was introduced in Ogre 1.12.10
  // and only fixed in Ogre 13.4.3
  // See https://github.com/OGRECave/ogre/pull/2519
  ASSERT_EQ(textureImage.getWidth(), expectedImage.getWidth());
  ASSERT_EQ(textureImage.getHeight(), expectedImage.getHeight());
#endif  // OGRE_MIN_VERSION(13, 4, 3)
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

#if OGRE_MIN_VERSION(13, 4, 3)
  ASSERT_THAT(
    std::vector<uint8_t>(textureImage.getData(), textureImage.getData() + textureImage.getSize()),
    testing::ElementsAreArray(testImage.getData(), testImage.getSize()));
#else
  // Can't compare the two images directly because the of a bug that was introduced in Ogre 1.12.10
  // and only fixed in Ogre 13.4.3
  // See https://github.com/OGRECave/ogre/pull/2519
  ASSERT_EQ(textureImage.getWidth(), testImage.getWidth());
  ASSERT_EQ(textureImage.getHeight(), testImage.getHeight());
#endif  // OGRE_MIN_VERSION(13, 4, 3)
}

TEST_F(RosImageTextureTestFixture, destructor_removes_the_texture_from_the_manager) {
  // Each ROSImageTexture instance creates a uniquely-named Ogre texture; if
  // the destructor failed to remove it, repeatedly allocating displays in a
  // session would leak GPU memory.
  std::string name;
  {
    ROSImageTexture texture;
    name = texture.getTexture()->getName();
    ASSERT_TRUE(Ogre::TextureManager::getSingleton().resourceExists(name, "rviz_rendering"));
  }
  ASSERT_FALSE(Ogre::TextureManager::getSingleton().resourceExists(name, "rviz_rendering"));
}

TEST_F(RosImageTextureTestFixture, default_construction_does_not_allocate_a_mipmap_chain) {
  ROSImageTexture texture;
  ASSERT_EQ(texture.getTexture()->getNumMipmaps(), 0u);
}

TEST_F(RosImageTextureTestFixture, enabling_smooth_scaling_allocates_a_mipmap_chain) {
  ROSImageTexture texture;
  texture.setSmoothScaling(true);

  // Ogre's GL3+ backend clamps the requested mip count to
  // floor(log2(max(w, h))) — getNumMipmaps() returns the number of levels
  // above level 0, so this is the count we expect to land on regardless of
  // what sentinel we passed in. Asserting the exact value pins behaviour
  // that a plain ASSERT_GT(..., 0u) would let drift (e.g. silently treating
  // (uint32)-1 as "all the mips").
  const auto tex = texture.getTexture();
  const auto expected_mips = static_cast<uint32_t>(
    std::floor(std::log2(std::max(tex->getWidth(), tex->getHeight()))));
  ASSERT_EQ(tex->getNumMipmaps(), expected_mips);
}

TEST_F(RosImageTextureTestFixture, toggling_smooth_scaling_preserves_the_texture_pointer) {
  // Displays bind to this texture by name; Ogre::TextureUnitState caches the
  // resolved TexturePtr after first lookup, so the pointer must be stable.
  ROSImageTexture texture;
  const auto * original = texture.getTexture().get();

  texture.setSmoothScaling(true);
  ASSERT_EQ(texture.getTexture().get(), original);

  texture.setSmoothScaling(false);
  ASSERT_EQ(texture.getTexture().get(), original);
}

TEST_F(RosImageTextureTestFixture, toggling_smooth_scaling_with_a_held_image_re_uploads_it) {
  // Regression: an earlier implementation overwrote the held frame with the
  // no_image.png placeholder on toggle, leaving the live image lost until
  // the next ROS message arrived — indefinite on latched topics.
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
  ASSERT_TRUE(texture.update());

  // No further messages — toggling must arm a re-upload so the next
  // update() refreshes the texture rather than no-oping.
  texture.setSmoothScaling(true);
  ASSERT_TRUE(texture.update());

  // And the result must still match the source image, not the placeholder.
  Ogre::Image textureImage;
  texture.getTexture()->convertToImage(textureImage);
  ASSERT_EQ(textureImage.getWidth(), testImage.getWidth());
  ASSERT_EQ(textureImage.getHeight(), testImage.getHeight());

#if OGRE_MIN_VERSION(13, 4, 3)
  ASSERT_THAT(
    std::vector<uint8_t>(textureImage.getData(), textureImage.getData() + textureImage.getSize()),
    testing::ElementsAreArray(testImage.getData(), testImage.getSize()));
#endif  // OGRE_MIN_VERSION(13, 4, 3) — see update_writes_new_image_to_the_texture
}

TEST_F(RosImageTextureTestFixture, update_with_smooth_scaling_writes_new_image_to_the_texture) {
  Ogre::Image testImage;
  testImage.load("test_20x20.png", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

  sensor_msgs::msg::Image::SharedPtr msg = std::make_shared<sensor_msgs::msg::Image>();
  msg->width = testImage.getWidth();
  msg->height = testImage.getHeight();
  msg->encoding = sensor_msgs::image_encodings::RGB8;
  msg->data = std::vector<uint8_t>(
    testImage.getData(), testImage.getData() + testImage.getSize());

  ROSImageTexture texture;
  texture.setSmoothScaling(true);
  texture.addMessage(msg);
  texture.update();

  Ogre::TexturePtr ogreTexture = texture.getTexture();
  ASSERT_GT(ogreTexture->getNumMipmaps(), 0u);

  Ogre::Image textureImage;
  ogreTexture->convertToImage(textureImage);

#if OGRE_MIN_VERSION(13, 4, 3)
  ASSERT_THAT(
    std::vector<uint8_t>(textureImage.getData(), textureImage.getData() + textureImage.getSize()),
    testing::ElementsAreArray(testImage.getData(), testImage.getSize()));
#else
  // Can't compare the two images directly because the of a bug that was introduced in Ogre 1.12.10
  // and only fixed in Ogre 13.4.3
  // See https://github.com/OGRECave/ogre/pull/2519
  ASSERT_EQ(textureImage.getWidth(), testImage.getWidth());
  ASSERT_EQ(textureImage.getHeight(), testImage.getHeight());
#endif  // OGRE_MIN_VERSION(13, 4, 3)

  // Verify mip level 1 was actually populated, not just allocated. Reading the
  // buffer back catches a TU_AUTOMIPMAP regression where the driver allocates
  // the chain but never generates content — which getNumMipmaps() alone, and
  // the convertToImage check above (level 0 only), would both miss.
  const auto & mip1 = ogreTexture->getBuffer(0, 1);
  ASSERT_EQ(mip1->getWidth(), testImage.getWidth() / 2u);
  ASSERT_EQ(mip1->getHeight(), testImage.getHeight() / 2u);
  std::vector<uint8_t> mip1_data(mip1->getWidth() * mip1->getHeight() * 3u);
  Ogre::PixelBox dst(mip1->getWidth(), mip1->getHeight(), 1, Ogre::PF_BYTE_RGB, mip1_data.data());
  mip1->blitToMemory(dst);
  ASSERT_TRUE(std::any_of(mip1_data.begin(), mip1_data.end(), [](uint8_t v) {return v != 0;}));
}

// ----- Bayer demosaic tests -----
//
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
  // Helper is only called with known Bayer encodings. Anything else is a
  // test-side bug.
  ADD_FAILURE() << "offsetsFor() called with unknown encoding: " << encoding;
  return {0, 0, 1, 1};
}

// Build a mosaic of a uniform scene with given (R, G, B) channel intensities,
// for the given Bayer layout. The mosaic places the R value at R positions,
// the G value at G positions, and the B value at B positions — matching what
// a real sensor records for a uniform scene. T is the per-sensel storage type
// (uint8_t for 8-bit encodings, uint16_t for 16-bit).
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
// strip the alpha channel here so callers can index uniformly with 3 bytes
// per pixel.
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

// Index helper for an h*w*3 flat buffer.
size_t rgbIndex(uint32_t y, uint32_t x, uint32_t width)
{
  return (static_cast<size_t>(y) * width + x) * 3;
}

// Expected sRGB output (8-bit) for a normalized linear input
uint8_t srgbEncodeByte(double linear)
{
  double s;
  if (linear <= 0.0031308) {
    s = 12.92 * linear;
  } else {
    s = 1.055 * std::pow(linear, 1.0 / 2.4) - 0.055;
  }
  if (s < 0.0) {s = 0.0;}
  if (s > 1.0) {s = 1.0;}
  return static_cast<uint8_t>(std::lround(s * 255.0));
}

}  // namespace

// Per-layout discriminating test helper: a mosaic where only `channel` is
// non-zero (channel 0 = R, 1 = G, 2 = B) must produce output where only that
// channel is populated (after sRGB encoding). A layout-swap bug surfaces as
// swapped output channels; per-layout mosaic construction makes it visible.
void checkPureChannelDemosaicSrgb(const std::string & encoding, int channel)
{
  ASSERT_GE(channel, 0);
  ASSERT_LE(channel, 2);
  const uint32_t w = 8;
  const uint32_t h = 8;
  const uint8_t v = 200;  // mid-range so sRGB encoding is meaningful

  const uint8_t r_val = (channel == 0) ? v : 0;
  const uint8_t g_val = (channel == 1) ? v : 0;
  const uint8_t b_val = (channel == 2) ? v : 0;

  auto msg = makeImage(w, h, encoding,
    buildUniformMosaic<uint8_t>(w, h, r_val, g_val, b_val, encoding), 1);

  ROSImageTexture texture;
  texture.setLinearInput(true);
  texture.addMessage(msg);
  ASSERT_TRUE(texture.update()) << "for encoding " << encoding;

  const std::vector<uint8_t> rgb = readTextureRGB(texture);
  ASSERT_EQ(rgb.size(), static_cast<size_t>(w) * h * 3) << "for " << encoding;

  const uint8_t expected = srgbEncodeByte(v / 255.0);

  // Check interior pixels. Edge / corner pixels are excluded because the
  // border passes interpolate over fewer neighbours and the result is less
  // exact for non-trivial neighbour patterns.
  for (uint32_t y = 2; y + 2 < h; ++y) {
    for (uint32_t x = 2; x + 2 < w; ++x) {
      const size_t i = rgbIndex(y, x, w);
      for (int c = 0; c < 3; ++c) {
        if (c == channel) {
          EXPECT_NEAR(rgb[i + c], expected, 2)
            << encoding << " channel=" << c << " at (" << y << "," << x << ")";
        } else {
          EXPECT_EQ(rgb[i + c], 0u)
            << encoding << " channel=" << c << " at (" << y << "," << x << ")";
        }
      }
    }
  }
}

// Bayer 8-bit sRGB path, parameterised over (layout, channel). Each
// combination becomes a named test in the failure output.
class Bayer8bitSrgbPureChannelTestFixture
  : public RosImageTextureTestFixture,
  public ::testing::WithParamInterface<std::tuple<std::string, int>>
{};

TEST_P(Bayer8bitSrgbPureChannelTestFixture, pure_channel_output_matches_expectation) {
  const auto & [encoding, channel] = GetParam();
  checkPureChannelDemosaicSrgb(encoding, channel);
}

INSTANTIATE_TEST_SUITE_P(
  AllLayoutsAndChannels,
  Bayer8bitSrgbPureChannelTestFixture,
  ::testing::Combine(
    ::testing::Values(
      sensor_msgs::image_encodings::BAYER_RGGB8,
      sensor_msgs::image_encodings::BAYER_BGGR8,
      sensor_msgs::image_encodings::BAYER_GBRG8,
      sensor_msgs::image_encodings::BAYER_GRBG8),
    ::testing::Values(0, 2)  // R and B are the discriminating channels
));

// 16-bit path: with Normalize Range = false and Max Value set to the input
// maximum, a "pure red" 16-bit mosaic produces pure red 8-bit output.
// Parameterised over the four layouts via four TEST_F entries: the demosaic
// implementation is parameterised on BayerLayout, so one test per layout is
// enough to catch a dispatch / channel-mapping bug at 16-bit.
void checkBayer16WithFixedMax(const std::string & encoding)
{
  const uint32_t w = 8;
  const uint32_t h = 8;
  const uint16_t r = 1023;  // simulate 10-bit-in-16 camera at full white

  auto msg = makeImage(w, h, encoding,
    buildUniformMosaic<uint16_t>(w, h, r, /*g*/ 0, /*b*/ 0, encoding), 2);

  ROSImageTexture texture;
  texture.setLinearInput(true);
  texture.setNormalizeFloatImage(false, 0.0, 1023.0);
  texture.addMessage(msg);
  ASSERT_TRUE(texture.update()) << "encoding " << encoding;

  const std::vector<uint8_t> rgb = readTextureRGB(texture);
  const uint8_t expected_r = 255;  // 1023 -> normalized 1.0 -> sRGB(1.0) = 255

  for (uint32_t y = 2; y + 2 < h; ++y) {
    for (uint32_t x = 2; x + 2 < w; ++x) {
      const size_t i = rgbIndex(y, x, w);
      EXPECT_NEAR(rgb[i + 0], expected_r, 1) << encoding << " (" << y << "," << x << ")";
      EXPECT_EQ(rgb[i + 1], 0u) << encoding << " (" << y << "," << x << ")";
      EXPECT_EQ(rgb[i + 2], 0u) << encoding << " (" << y << "," << x << ")";
    }
  }
}

class Bayer16bitSrgbFixedMaxTestFixture
  : public RosImageTextureTestFixture,
  public ::testing::WithParamInterface<std::string>
{};

TEST_P(Bayer16bitSrgbFixedMaxTestFixture, pure_red_output_matches_expectation) {
  checkBayer16WithFixedMax(GetParam());
}

INSTANTIATE_TEST_SUITE_P(
  AllLayouts,
  Bayer16bitSrgbFixedMaxTestFixture,
  ::testing::Values(
    sensor_msgs::image_encodings::BAYER_RGGB16,
    sensor_msgs::image_encodings::BAYER_BGGR16,
    sensor_msgs::image_encodings::BAYER_GBRG16,
    sensor_msgs::image_encodings::BAYER_GRBG16));

// 16-bit path: a mosaic at half the user-specified Max Value (10-bit half-
// white = 511) should produce sRGB(0.5) ~= 188, not 128 (which would be the
// result of sRGB-then-scale instead of scale-then-sRGB).
TEST_F(RosImageTextureTestFixture, bayer_16bit_srgb_after_scaling) {
  const std::string encoding = sensor_msgs::image_encodings::BAYER_RGGB16;
  const uint32_t w = 8;
  const uint32_t h = 8;
  const uint16_t r = 511;  // ~half of 1023

  auto msg = makeImage(w, h, encoding,
    buildUniformMosaic<uint16_t>(w, h, r, /*g*/ 0, /*b*/ 0, encoding), 2);

  ROSImageTexture texture;
  texture.setLinearInput(true);
  texture.setNormalizeFloatImage(false, 0.0, 1023.0);
  texture.addMessage(msg);
  ASSERT_TRUE(texture.update());

  const std::vector<uint8_t> rgb = readTextureRGB(texture);
  const uint8_t expected_r = srgbEncodeByte(511.0 / 1023.0);  // ~= 188

  // The expected ~188 also discriminates against sRGB-then-scale, which
  // would produce ~2/255 (sRGB(511/65535) * 1023/65535).
  for (uint32_t y = 2; y + 2 < h; ++y) {
    for (uint32_t x = 2; x + 2 < w; ++x) {
      const size_t i = rgbIndex(y, x, w);
      EXPECT_NEAR(rgb[i + 0], expected_r, 2) << "at (" << y << "," << x << ")";
    }
  }
}

// 16-bit path with Min Value (black point) > 0: with Min=100, Max=1100, an
// input of 100 should produce 0.
TEST_F(RosImageTextureTestFixture, bayer_16bit_input_at_min_value_maps_to_black) {
  const std::string encoding = sensor_msgs::image_encodings::BAYER_RGGB16;
  const uint32_t w = 8;
  const uint32_t h = 8;

  ROSImageTexture texture;
  texture.setNormalizeFloatImage(false, 100.0, 1100.0);

  auto msg = makeImage(w, h, encoding,
    buildUniformMosaic<uint16_t>(w, h, 100, 100, 100, encoding), 2);
  texture.addMessage(msg);
  ASSERT_TRUE(texture.update());
  const std::vector<uint8_t> rgb = readTextureRGB(texture);
  for (uint32_t y = 2; y + 2 < h; ++y) {
    for (uint32_t x = 2; x + 2 < w; ++x) {
      const size_t i = rgbIndex(y, x, w);
      EXPECT_EQ(rgb[i + 0], 0u);
      EXPECT_EQ(rgb[i + 1], 0u);
      EXPECT_EQ(rgb[i + 2], 0u);
    }
  }
}

// Counterpart to the black-point test: an input at Max Value (1100) should
// produce 255.
TEST_F(RosImageTextureTestFixture, bayer_16bit_input_at_max_value_maps_to_white) {
  const std::string encoding = sensor_msgs::image_encodings::BAYER_RGGB16;
  const uint32_t w = 8;
  const uint32_t h = 8;

  ROSImageTexture texture;
  texture.setNormalizeFloatImage(false, 100.0, 1100.0);

  auto msg = makeImage(w, h, encoding,
    buildUniformMosaic<uint16_t>(w, h, 1100, 1100, 1100, encoding), 2);
  texture.addMessage(msg);
  ASSERT_TRUE(texture.update());
  const std::vector<uint8_t> rgb = readTextureRGB(texture);
  for (uint32_t y = 2; y + 2 < h; ++y) {
    for (uint32_t x = 2; x + 2 < w; ++x) {
      const size_t i = rgbIndex(y, x, w);
      EXPECT_EQ(rgb[i + 0], 255u);
      EXPECT_EQ(rgb[i + 1], 255u);
      EXPECT_EQ(rgb[i + 2], 255u);
    }
  }
}

// sRGB transfer-function anchor points, verified end-to-end via the demosaic
// output rather than by cross-checking the test-side reference. Multiple
// anchors catch a wrong-exponent bug that a single midpoint could miss.
// Note: the piecewise linear segment (linear <= 0.0031308) is not exercised
// here — 1/255 > 0.0031308, so 8-bit input can't reach it, and the existing
// 16-bit tests all sit far above the knee.
TEST_F(RosImageTextureTestFixture, srgb_transfer_anchor_points_via_demosaic_output) {
  const std::string encoding = sensor_msgs::image_encodings::BAYER_RGGB8;
  const uint32_t w = 6;
  const uint32_t h = 6;

  struct Anchor
  {
    uint8_t input;
    uint8_t expected;
    const char * name;
  };
  // Expected values follow IEC 61966-2-1 evaluated at input / 255.0.
  const std::vector<Anchor> anchors = {
    {0, 0, "black"},
    {1, 13, "near knee"},    // exponent branch, near the piecewise boundary
    {128, 188, "midtone"},   // overall exponent/gain/offset
    {255, 255, "white"},
  };

  for (const auto & a : anchors) {
    std::vector<uint8_t> data(static_cast<size_t>(w) * h, a.input);
    auto msg = makeImage(w, h, encoding, std::move(data), 1);

    ROSImageTexture texture;
    texture.setLinearInput(true);
    texture.addMessage(msg);
    ASSERT_TRUE(texture.update()) << a.name;

    const std::vector<uint8_t> rgb = readTextureRGB(texture);
    for (uint32_t y = 2; y + 2 < h; ++y) {
      for (uint32_t x = 2; x + 2 < w; ++x) {
        const size_t i = rgbIndex(y, x, w);
        EXPECT_NEAR(rgb[i + 0], a.expected, 1) << a.name;
        EXPECT_NEAR(rgb[i + 1], a.expected, 1) << a.name;
        EXPECT_NEAR(rgb[i + 2], a.expected, 1) << a.name;
      }
    }
  }
}

// Default (Treat as linear = false): 8-bit Bayer is passed through the linear
// rescale straight to 8-bit — no sRGB gamma is applied. A pure-red mosaic
// with r = 200 must therefore produce output r = 200 exactly (not sRGB(200)
// = ~219), which discriminates the default from the sRGB-encoded path.
// One layout is enough — layout dispatch is exercised end-to-end by the
// 8-bit sRGB matrix, and the quantizer choice (linear vs sRGB) is orthogonal
// to the layout template parameter in demosaicBayerImpl.
TEST_F(RosImageTextureTestFixture, bayer_8bit_default_is_linear_passthrough) {
  const std::string encoding = sensor_msgs::image_encodings::BAYER_RGGB8;
  const uint32_t w = 8;
  const uint32_t h = 8;
  const uint8_t r = 200;

  auto msg = makeImage(w, h, encoding,
    buildUniformMosaic<uint8_t>(w, h, r, /*g*/ 0, /*b*/ 0, encoding), 1);

  ROSImageTexture texture;  // linear_input_ defaults to false
  texture.addMessage(msg);
  ASSERT_TRUE(texture.update());

  const std::vector<uint8_t> rgb = readTextureRGB(texture);
  for (uint32_t y = 2; y + 2 < h; ++y) {
    for (uint32_t x = 2; x + 2 < w; ++x) {
      const size_t i = rgbIndex(y, x, w);
      EXPECT_EQ(rgb[i + 0], r) << "at (" << y << "," << x << ")";
      EXPECT_EQ(rgb[i + 1], 0u) << "at (" << y << "," << x << ")";
      EXPECT_EQ(rgb[i + 2], 0u) << "at (" << y << "," << x << ")";
    }
  }
}

// Default (Treat as linear = false): 16-bit Bayer is linearly rescaled to
// 8-bit — no gamma. A mid-range input (r = 511 out of Max = 1023) must
// produce r ~= 127, not sRGB(511/1023) ~= 188. Linear-mode counterpart to
// bayer_16bit_srgb_after_scaling. One layout is enough — 16-bit layout
// dispatch is covered by the sRGB matrix; this test only discriminates
// linear vs sRGB quantization.
TEST_F(RosImageTextureTestFixture, bayer_16bit_default_is_linear_passthrough) {
  const std::string encoding = sensor_msgs::image_encodings::BAYER_RGGB16;
  const uint32_t w = 8;
  const uint32_t h = 8;
  const uint16_t r = 511;

  auto msg = makeImage(w, h, encoding,
    buildUniformMosaic<uint16_t>(w, h, r, /*g*/ 0, /*b*/ 0, encoding), 2);

  ROSImageTexture texture;  // linear_input_ defaults to false
  texture.setNormalizeFloatImage(false, 0.0, 1023.0);
  texture.addMessage(msg);
  ASSERT_TRUE(texture.update());

  const std::vector<uint8_t> rgb = readTextureRGB(texture);
  const uint8_t expected_r = static_cast<uint8_t>(std::lround(511.0 * 255.0 / 1023.0));

  // The expected ~127 also discriminates against sRGB mode (~188).
  for (uint32_t y = 2; y + 2 < h; ++y) {
    for (uint32_t x = 2; x + 2 < w; ++x) {
      const size_t i = rgbIndex(y, x, w);
      EXPECT_NEAR(rgb[i + 0], expected_r, 1) << "at (" << y << "," << x << ")";
    }
  }
}

// 16-bit Bayer with normalize=false and a fixed max deliberately *larger*
// than the actual peak input: the fixed max must be respected. This
// discriminates the "use fixed bounds" path from an accidental auto-detect
// (which would remap the actual 1023 peak to 255 regardless of the user's
// Max Value setting).
TEST_F(RosImageTextureTestFixture, bayer_16bit_fixed_max_above_actual_is_respected) {
  const std::string encoding = sensor_msgs::image_encodings::BAYER_RGGB16;
  const uint32_t w = 8;
  const uint32_t h = 8;
  const uint16_t r = 1023;  // actual peak

  auto msg = makeImage(w, h, encoding,
    buildUniformMosaic<uint16_t>(w, h, r, /*g*/ 0, /*b*/ 0, encoding), 2);

  ROSImageTexture texture;  // linear passthrough
  texture.setNormalizeFloatImage(false, 0.0, 4095.0);  // user says 12-bit range
  texture.addMessage(msg);
  ASSERT_TRUE(texture.update());

  const std::vector<uint8_t> rgb = readTextureRGB(texture);
  // 1023 / 4095 * 255 ~= 63. Auto-detected max (1023) would give 255.
  const uint8_t expected_r = static_cast<uint8_t>(std::lround(1023.0 * 255.0 / 4095.0));

  for (uint32_t y = 2; y + 2 < h; ++y) {
    for (uint32_t x = 2; x + 2 < w; ++x) {
      const size_t i = rgbIndex(y, x, w);
      EXPECT_NEAR(rgb[i + 0], expected_r, 1) << "at (" << y << "," << x << ")";
    }
  }
}

// Toggling linear_input at runtime must re-run the converter on the held
// frame — the same latched-topic contract that setSmoothScaling honors.
TEST_F(RosImageTextureTestFixture, toggling_linear_input_reprocesses_held_frame) {
  const std::string encoding = sensor_msgs::image_encodings::BAYER_RGGB8;
  const uint32_t w = 8;
  const uint32_t h = 8;
  const uint8_t r = 200;

  auto msg = makeImage(w, h, encoding,
    buildUniformMosaic<uint8_t>(w, h, r, 0, 0, encoding), 1);

  ROSImageTexture texture;
  texture.addMessage(msg);
  ASSERT_TRUE(texture.update());

  // Baseline: linear pass-through (default).
  {
    const std::vector<uint8_t> rgb = readTextureRGB(texture);
    const size_t i = rgbIndex(4, 4, w);
    EXPECT_EQ(rgb[i + 0], r);
  }

  // Toggle on without a new message — next update() must re-encode.
  texture.setLinearInput(true);
  ASSERT_TRUE(texture.update());
  {
    const std::vector<uint8_t> rgb = readTextureRGB(texture);
    const size_t i = rgbIndex(4, 4, w);
    EXPECT_NEAR(rgb[i + 0], srgbEncodeByte(r / 255.0), 2);
  }

  // Toggle back off — must re-encode back to the linear value.
  texture.setLinearInput(false);
  ASSERT_TRUE(texture.update());
  {
    const std::vector<uint8_t> rgb = readTextureRGB(texture);
    const size_t i = rgbIndex(4, 4, w);
    EXPECT_EQ(rgb[i + 0], r);
  }
}

// Median-frames path on 16-bit Bayer: with a running window of 5 frames,
// feed four high-peak frames followed by one low-peak frame. Median max is
// then still the high peak, so the low-peak frame is normalised against the
// median (high) rather than its own peak — the low-peak red pixels appear
// dim, not saturated. Behaviour discriminator: with median filtering, the
// interior red is around low_peak * 255 / median (~= 32); without it, red
// would saturate at 255.
TEST_F(RosImageTextureTestFixture, bayer_16bit_median_frames_tracks_median_not_last) {
  const std::string encoding = sensor_msgs::image_encodings::BAYER_RGGB16;
  const uint32_t w = 8;
  const uint32_t h = 8;
  const uint16_t high_peak = 4095;
  const uint16_t low_peak = 511;

  ROSImageTexture texture;
  texture.setNormalizeFloatImage(true);
  texture.setMedianFrames(5);

  // Four high frames + one low: sorted maxes = [511, 4095, 4095, 4095, 4095],
  // median = 4095.
  for (uint16_t peak : {high_peak, high_peak, high_peak, high_peak, low_peak}) {
    auto msg = makeImage(w, h, encoding,
      buildUniformMosaic<uint16_t>(w, h, peak, /*g*/ 0, /*b*/ 0, encoding), 2);
    texture.addMessage(msg);
    ASSERT_TRUE(texture.update());
  }

  const std::vector<uint8_t> rgb = readTextureRGB(texture);
  const uint8_t expected_r =
    static_cast<uint8_t>(std::lround(low_peak * 255.0 / high_peak));  // ~= 32
  ASSERT_LT(expected_r, 60u);  // discriminates against no-median (255)

  for (uint32_t y = 2; y + 2 < h; ++y) {
    for (uint32_t x = 2; x + 2 < w; ++x) {
      const size_t i = rgbIndex(y, x, w);
      EXPECT_NEAR(rgb[i + 0], expected_r, 3) << "at (" << y << "," << x << ")";
    }
  }
}

// Small-image smoke tests: 2x2 and 3x3 inputs must not crash or produce NaN.
// These exercise the border-only path and the corner cases of the bilinear
// neighbour averaging (count of valid neighbours < 4).
TEST_F(RosImageTextureTestFixture, small_2x2_image_borders_produce_expected_colour) {
  // Pure-red mosaic on the smallest possible image: every output pixel is
  // computed by the border handler with fewer than 4 valid neighbours. All
  // R samples equal 200; all G and B sensels are 0. Every output pixel
  // should therefore show R dominant and near 200, with G and B near 0.
  const std::string encoding = sensor_msgs::image_encodings::BAYER_RGGB8;
  const uint8_t r = 200;
  auto msg = makeImage(
    2, 2, encoding, buildUniformMosaic<uint8_t>(2, 2, r, /*g*/ 0, /*b*/ 0, encoding), 1);

  ROSImageTexture texture;
  texture.addMessage(msg);
  ASSERT_TRUE(texture.update());

  const std::vector<uint8_t> rgb = readTextureRGB(texture);
  ASSERT_EQ(rgb.size(), 2u * 2u * 3u);
  for (uint32_t y = 0; y < 2; ++y) {
    for (uint32_t x = 0; x < 2; ++x) {
      const size_t i = rgbIndex(y, x, 2);
      EXPECT_GE(rgb[i + 0], 100u) << "R at (" << y << "," << x << ")";
      EXPECT_EQ(rgb[i + 1], 0u) << "G at (" << y << "," << x << ")";
      EXPECT_EQ(rgb[i + 2], 0u) << "B at (" << y << "," << x << ")";
    }
  }
}

TEST_F(RosImageTextureTestFixture, small_3x3_image_borders_produce_expected_colour) {
  // Same idea on a 3x3 BGGR: the centre pixel gets the full-neighbourhood
  // path; the 8 border pixels exercise the corner / edge branches of the
  // handler. All R samples equal 200; G and B are 0; output R should be
  // dominant everywhere.
  const std::string encoding = sensor_msgs::image_encodings::BAYER_BGGR8;
  const uint8_t r = 200;
  auto msg = makeImage(
    3, 3, encoding, buildUniformMosaic<uint8_t>(3, 3, r, /*g*/ 0, /*b*/ 0, encoding), 1);

  ROSImageTexture texture;
  texture.addMessage(msg);
  ASSERT_TRUE(texture.update());

  const std::vector<uint8_t> rgb = readTextureRGB(texture);
  ASSERT_EQ(rgb.size(), 3u * 3u * 3u);
  for (uint32_t y = 0; y < 3; ++y) {
    for (uint32_t x = 0; x < 3; ++x) {
      const size_t i = rgbIndex(y, x, 3);
      EXPECT_GE(rgb[i + 0], 100u) << "R at (" << y << "," << x << ")";
      EXPECT_EQ(rgb[i + 1], 0u) << "G at (" << y << "," << x << ")";
      EXPECT_EQ(rgb[i + 2], 0u) << "B at (" << y << "," << x << ")";
    }
  }
}

// Unknown bayer_* encoding is rejected via UnsupportedImageEncoding. The
// exception propagates out of texture.update(); the display layer catches it
// at image_display.cpp / camera_display.cpp and sets an error status.
TEST_F(RosImageTextureTestFixture, unknown_bayer_encoding_throws_via_update) {
  auto msg = makeImage(4, 4, "bayer_xyzzy8", std::vector<uint8_t>(16, 0u), 1);

  ROSImageTexture texture;
  texture.addMessage(msg);

  EXPECT_THROW(texture.update(), UnsupportedImageEncoding);
}

// Validation: an oversize message dimension is rejected upfront rather than
// allowed through to the demosaic loops.
TEST_F(RosImageTextureTestFixture, oversize_dimension_rejected) {
  auto msg = std::make_shared<sensor_msgs::msg::Image>();
  msg->encoding = sensor_msgs::image_encodings::BAYER_RGGB8;
  msg->width = 100000u;
  msg->height = 100000u;
  msg->step = msg->width;
  msg->data = std::vector<uint8_t>(16, 0u);  // deliberately too small

  ROSImageTexture texture;
  texture.addMessage(msg);
  EXPECT_THROW(texture.update(), MalformedImageMessage);
}

// Validation: a truncated data buffer (smaller than height * step) is rejected.
TEST_F(RosImageTextureTestFixture, truncated_data_rejected) {
  auto msg = std::make_shared<sensor_msgs::msg::Image>();
  msg->encoding = sensor_msgs::image_encodings::BAYER_RGGB8;
  msg->width = 8;
  msg->height = 8;
  msg->step = 8;
  msg->data = std::vector<uint8_t>(8, 0u);  // 8 bytes but expects 64

  ROSImageTexture texture;
  texture.addMessage(msg);
  EXPECT_THROW(texture.update(), MalformedImageMessage);
}

// Padded-stride 16-bit Bayer: a message where step > width * 2 (extra
// padding bytes at the end of each row, as some drivers emit for SIMD
// alignment) must still demosaic correctly — the buffer is repacked in
// update() before the converter sees it. Padding bytes are 0xFF: if the
// repack ever skipped a row (or was bypassed and padding leaked into the
// min/max scan), max_value would jump to 65535 and the actual red pixels
// (1023) would map to a near-black ~4/255. Full red (>= 250) is therefore
// a strong discriminator that the repack ran end-to-end.
TEST_F(RosImageTextureTestFixture, bayer_16bit_padded_stride) {
  const std::string encoding = sensor_msgs::image_encodings::BAYER_RGGB16;
  const uint32_t w = 8;
  const uint32_t h = 8;
  const uint32_t step = 24;  // 8 pixels * 2 bytes + 8 bytes of padding per row

  std::vector<uint8_t> tight = buildUniformMosaic<uint16_t>(
    w, h, /*r*/ 1023, /*g*/ 0, /*b*/ 0, encoding);
  std::vector<uint8_t> padded(static_cast<size_t>(step) * h, uint8_t{0xFF});
  for (uint32_t y = 0; y < h; ++y) {
    std::memcpy(
      padded.data() + static_cast<size_t>(y) * step,
      tight.data() + static_cast<size_t>(y) * w * 2u,
      w * 2u);
  }

  auto msg = std::make_shared<sensor_msgs::msg::Image>();
  msg->width = w;
  msg->height = h;
  msg->encoding = encoding;
  msg->step = step;
  msg->data = std::move(padded);

  ROSImageTexture texture;
  texture.setNormalizeFloatImage(true);  // auto-normalise; default also true
  texture.setMedianFrames(1);            // single frame -> running median == raw min/max
  texture.addMessage(msg);
  ASSERT_TRUE(texture.update());

  const std::vector<uint8_t> rgb = readTextureRGB(texture);
  for (uint32_t y = 2; y + 2 < h; ++y) {
    for (uint32_t x = 2; x + 2 < w; ++x) {
      const size_t i = rgbIndex(y, x, w);
      // Red pixels saturate (max_value == 1023 from the actual R samples).
      EXPECT_GE(rgb[i + 0], 250u) << "at (" << y << "," << x << ")";
      // G and B sensels are zero, so the demosaiced G/B output is zero
      // regardless of the normalisation range.
      EXPECT_LE(rgb[i + 1], 5u) << "at (" << y << "," << x << ")";
      EXPECT_LE(rgb[i + 2], 5u) << "at (" << y << "," << x << ")";
    }
  }
}
