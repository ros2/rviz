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

#include "rviz_default_plugins/displays/map/palette_builder.hpp"

#include <cstring>
#include <memory>
#include <vector>

namespace rviz_default_plugins
{
namespace displays
{

PaletteBuilder::PaletteBuilder()
{
  palette_ = std::vector<unsigned char>(256 * 4, 0);
}

std::shared_ptr<PaletteBuilder> PaletteBuilder::setColorForIllegalPositiveValues(
  unsigned char r, unsigned char g, unsigned char b)
{
  for (unsigned char i = 101; i <= 127; i++) {
    setColorForValue(i, r, g, b, 255);
  }
  return shared_from_this();
}

std::shared_ptr<PaletteBuilder> PaletteBuilder::setRedYellowColorsForIllegalNegativeValues()
{
  for (unsigned char i = 128; i <= 254; i++) {
    // set shades from red to yellow
    setColorForValue(i, 255, (255 * (i - 128)) / (254 - 128), 0, 255);
  }
  return shared_from_this();
}

std::shared_ptr<PaletteBuilder> PaletteBuilder::setColorForLegalNegativeValueMinusOne(
  unsigned char r, unsigned char g, unsigned char b)
{
  setColorForValue(255, r, g, b, 255);
  return shared_from_this();
}

std::shared_ptr<PaletteBuilder> PaletteBuilder::setColorForValue(
  unsigned char palette_position,
  unsigned char r, unsigned char g, unsigned char b, unsigned char alpha)
{
  palette_[4 * palette_position] = r;
  palette_[4 * palette_position + 1] = g;
  palette_[4 * palette_position + 2] = b;
  palette_[4 * palette_position + 3] = alpha;
  return shared_from_this();
}

std::vector<unsigned char> PaletteBuilder::buildPalette()
{
  return palette_;
}

std::vector<unsigned char> makeMapPalette(bool binary, int threshold)
{
  auto palette_builder = std::make_shared<PaletteBuilder>();
  if (binary) {
    for (unsigned char i = 0; i < threshold; i++) {
      palette_builder->setColorForValue(i, 255, 255, 255, 255);
    }
    for (unsigned char i = threshold; i <= 100; i++) {
      palette_builder->setColorForValue(i, 0, 0, 0, 255);
    }
  } else {
    for (unsigned char i = 0; i <= 100; i++) {
      // Standard gray map palette values
      unsigned char v = 255 - (255 * i) / 100;
      palette_builder->setColorForValue(i, v, v, v, 255);
    }
  }
  return palette_builder->setColorForIllegalPositiveValues(0, 255, 0)
         ->setRedYellowColorsForIllegalNegativeValues()
         ->setColorForLegalNegativeValueMinusOne(0x70, 0x89, 0x86)
         ->buildPalette();
}

std::vector<unsigned char> makeCostmapPalette(bool binary, int threshold)
{
  auto palette_builder = std::make_shared<PaletteBuilder>();
  if (binary) {
    for (unsigned char i = 0; i < threshold; i++) {
      palette_builder->setColorForValue(i, 0, 0, 255, 255);
    }
    for (unsigned char i = threshold; i <= 100; i++) {
      palette_builder->setColorForValue(i, 255, 0, 0, 255);
    }
  } else {
    palette_builder->setColorForValue(0, 0, 0, 0, 0);
    for (unsigned char i = 1; i <= 98; i++) {
      unsigned char v = (255 * i) / 100;
      palette_builder->setColorForValue(i, v, 0, 255 - v, 255);
    }
    palette_builder->setColorForValue(99, 0, 255, 255, 255);  // obstacle values in cyan
    palette_builder->setColorForValue(100, 255, 0, 255, 255);  // lethal obstacle values in purple
  }

  return palette_builder->setColorForIllegalPositiveValues(0, 255, 0)
         ->setRedYellowColorsForIllegalNegativeValues()
         ->setColorForLegalNegativeValueMinusOne(0x70, 0x89, 0x86)
         ->buildPalette();
}

std::vector<unsigned char> makeRawPalette(bool binary, int threshold)
{
  auto palette_builder = std::make_shared<PaletteBuilder>();
  if (binary) {
    for (unsigned char i = 0; i < threshold; i++) {
      palette_builder->setColorForValue(i, 0, 0, 0, 255);
    }
    for (int i = threshold; i < 256; i++) {
      palette_builder->setColorForValue(i, 255, 255, 255, 255);
    }
  } else {
    for (int i = 0; i < 256; i++) {
      palette_builder->setColorForValue(i, i, i, i, 255);
    }
  }
  return palette_builder->buildPalette();
}

}  // namespace displays
}  // namespace rviz_default_plugins
