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

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <memory>

#include "rviz_default_plugins/displays/map/palette_builder.hpp"

using namespace ::testing;  // NOLINT

TEST(PaletteBuilderTest, constructs_a_default_initialised_list) {
  auto palette_builder = std::make_shared<rviz_default_plugins::displays::PaletteBuilder>();

  auto palette = palette_builder->buildPalette();
  for (int i = 0; i < 4 * 256; i++) {
    EXPECT_THAT(palette[i], Eq(0));
  }
}

TEST(PaletteBuilderTest, setColorForValue_sets_value_at_correct_position) {
  auto palette_builder = std::make_shared<rviz_default_plugins::displays::PaletteBuilder>();

  auto palette = palette_builder->setColorForValue(1, 10, 20, 30, 40)->buildPalette();
  EXPECT_THAT(palette[4], Eq(10));
  EXPECT_THAT(palette[5], Eq(20));
  EXPECT_THAT(palette[6], Eq(30));
  EXPECT_THAT(palette[7], Eq(40));
}

TEST(PaletteBuilderTest, setColorForIllegalPositiveValues_sets_a_color_between_101_and_127) {
  auto palette_builder = std::make_shared<rviz_default_plugins::displays::PaletteBuilder>();

  auto palette = palette_builder->setColorForIllegalPositiveValues(10, 20, 30)->buildPalette();
  EXPECT_THAT(palette[403], Eq(0));
  for (int i = 0; i < 27; i++) {
    EXPECT_THAT(palette[404 + 4 * i], Eq(10));
    EXPECT_THAT(palette[404 + 4 * i + 1], Eq(20));
    EXPECT_THAT(palette[404 + 4 * i + 2], Eq(30));
    EXPECT_THAT(palette[404 + 4 * i + 3], Eq(255));
  }
  EXPECT_THAT(palette[512], Eq(0));
}

TEST(PaletteBuilderTest, setColorForIllegalNegativeValues_sets_a_color_between_128_and_254) {
  auto palette_builder = std::make_shared<rviz_default_plugins::displays::PaletteBuilder>();

  auto palette = palette_builder->setRedYellowColorsForIllegalNegativeValues()->buildPalette();
  EXPECT_THAT(palette[128 * 4 - 1], Eq(0));
  for (int i = 0; i < 127; i++) {
    EXPECT_THAT(palette[128 * 4 + 4 * i], Eq(255));
    EXPECT_THAT(palette[128 * 4 + 4 * i + 2], Eq(0));
    EXPECT_THAT(palette[128 * 4 + 4 * i + 3], Eq(255));
  }
  EXPECT_THAT(palette[255 * 4 + 1], Eq(0));
}

TEST(PaletteBuilderTest, setColorForLegalNegativeValueMinusOne_sets_a_color_for_MinusOne) {
  auto palette_builder = std::make_shared<rviz_default_plugins::displays::PaletteBuilder>();

  auto palette = palette_builder->setColorForLegalNegativeValueMinusOne(30, 40, 50)->buildPalette();
  EXPECT_THAT(palette[255 * 4 - 1], Eq(0));
  EXPECT_THAT(palette[255 * 4], Eq(30));
  EXPECT_THAT(palette[255 * 4 + 1], Eq(40));
  EXPECT_THAT(palette[255 * 4 + 2], Eq(50));
  EXPECT_THAT(palette[255 * 4 + 3], Eq(255));
}
