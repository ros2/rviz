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


#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__IMAGE__BAYER_FORMAT_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__IMAGE__BAYER_FORMAT_HPP_

#include <optional>
#include <string>

#include "sensor_msgs/image_encodings.hpp"

namespace rviz_default_plugins
{
namespace displays
{

// The position of R within the 2x2 Bayer cell uniquely identifies the layout.
// B sits at the opposite corner; G occupies the other two cells.
//   RGGB -> R at (0, 0)
//   BGGR -> R at (1, 1)
//   GBRG -> R at (1, 0)
//   GRBG -> R at (0, 1)
enum class BayerLayout
{
  RGGB,
  BGGR,
  GBRG,
  GRBG
};

constexpr int bayerRedRow(BayerLayout layout)
{
  return (layout == BayerLayout::BGGR || layout == BayerLayout::GBRG) ? 1 : 0;
}

constexpr int bayerRedCol(BayerLayout layout)
{
  return (layout == BayerLayout::BGGR || layout == BayerLayout::GRBG) ? 1 : 0;
}

struct BayerFormat
{
  BayerLayout layout;
  bool is_16bit;
};

// Single source of truth for the Bayer encodings the image displays can
// demosaic. Returns std::nullopt for anything else — including Bayer variants
// sensor_msgs may grow later: gating UI and conversion on this whitelist
// (rather than on image_encodings::isBayer()) keeps both in lockstep with
// what the converter actually implements.
inline std::optional<BayerFormat> bayerFormatFromEncoding(const std::string & encoding)
{
  if (encoding == sensor_msgs::image_encodings::BAYER_RGGB8) {
    return BayerFormat{BayerLayout::RGGB, false};
  }
  if (encoding == sensor_msgs::image_encodings::BAYER_BGGR8) {
    return BayerFormat{BayerLayout::BGGR, false};
  }
  if (encoding == sensor_msgs::image_encodings::BAYER_GBRG8) {
    return BayerFormat{BayerLayout::GBRG, false};
  }
  if (encoding == sensor_msgs::image_encodings::BAYER_GRBG8) {
    return BayerFormat{BayerLayout::GRBG, false};
  }
  if (encoding == sensor_msgs::image_encodings::BAYER_RGGB16) {
    return BayerFormat{BayerLayout::RGGB, true};
  }
  if (encoding == sensor_msgs::image_encodings::BAYER_BGGR16) {
    return BayerFormat{BayerLayout::BGGR, true};
  }
  if (encoding == sensor_msgs::image_encodings::BAYER_GBRG16) {
    return BayerFormat{BayerLayout::GBRG, true};
  }
  if (encoding == sensor_msgs::image_encodings::BAYER_GRBG16) {
    return BayerFormat{BayerLayout::GRBG, true};
  }
  return std::nullopt;
}

}  // namespace displays
}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__IMAGE__BAYER_FORMAT_HPP_
