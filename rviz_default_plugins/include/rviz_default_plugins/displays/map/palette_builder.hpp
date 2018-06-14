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

#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__MAP__PALETTE_BUILDER_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__MAP__PALETTE_BUILDER_HPP_

#include <memory>
#include <vector>

#include "rviz_default_plugins/visibility_control.hpp"

namespace rviz_default_plugins
{
namespace displays
{

std::vector<unsigned char> makeRawPalette();
std::vector<unsigned char> makeMapPalette();
std::vector<unsigned char> makeCostmapPalette();

class PaletteBuilder : public
  std::enable_shared_from_this<PaletteBuilder>
{
public:
  RVIZ_DEFAULT_PLUGINS_PUBLIC
  PaletteBuilder();

  RVIZ_DEFAULT_PLUGINS_PUBLIC
  virtual ~PaletteBuilder() = default;

  RVIZ_DEFAULT_PLUGINS_PUBLIC
  std::shared_ptr<PaletteBuilder> setColorForIllegalPositiveValues(
    unsigned char r, unsigned char g, unsigned char b);

  RVIZ_DEFAULT_PLUGINS_PUBLIC
  std::shared_ptr<PaletteBuilder> setRedYellowColorsForIllegalNegativeValues();

  RVIZ_DEFAULT_PLUGINS_PUBLIC
  std::shared_ptr<PaletteBuilder> setColorForLegalNegativeValueMinusOne(
    unsigned char r, unsigned char g, unsigned char b);

  RVIZ_DEFAULT_PLUGINS_PUBLIC
  std::shared_ptr<PaletteBuilder> setColorForValue(
    unsigned char palette_position,
    unsigned char r, unsigned char g, unsigned char b, unsigned char alpha);

  RVIZ_DEFAULT_PLUGINS_PUBLIC
  std::vector<unsigned char> buildPalette();

private:
  std::vector<unsigned char> palette_;
};

}  // namespace displays
}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__MAP__PALETTE_BUILDER_HPP_
