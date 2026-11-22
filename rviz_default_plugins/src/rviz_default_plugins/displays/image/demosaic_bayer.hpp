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


#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__IMAGE__DEMOSAIC_BAYER_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__IMAGE__DEMOSAIC_BAYER_HPP_

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "bayer_format.hpp"

namespace rviz_default_plugins
{
namespace displays
{
// Passthrough for 8-bit
struct IdentityQuantizer
{
  uint8_t operator()(int v) const {return static_cast<uint8_t>(v);}
};

// Map each value through a lookup table,
// which must have 1 << (8 * sizeof(T)) entries for input type T.
struct LutQuantizer
{
  const uint8_t * lut;
  uint8_t operator()(int v) const {return lut[v];}
};

// Demosaic implementation. T is the input pixel type (uint8_t or uint16_t).
// R_DY / R_DX encode the layout (position of R within the 2x2 cell).
// `quantize` maps a value of the input type to the uint8_t output.
template<typename T, int R_DY, int R_DX, typename Quantizer>
static void demosaicBayerImpl(
  uint8_t *dst_rgb, const T *src_mosaic, uint32_t height,
  uint32_t width, uint32_t stride_in_pixels,
  const Quantizer & quantize)
{
  constexpr int B_DY = 1 - R_DY;
  constexpr int B_DX = 1 - R_DX;

  const auto write_pixel = [&](uint32_t y, uint32_t x, int r, int g, int b) {
      const size_t out_idx = (static_cast<size_t>(y) * width + x) * 3;
      dst_rgb[out_idx + 0] = quantize(r);
      dst_rgb[out_idx + 1] = quantize(g);
      dst_rgb[out_idx + 2] = quantize(b);
    };

  // Border-pixel handler for the 1-pixel border
  // Use clamp-to-edge for missing neighbours.
  // For a G-pixel on the boundary, the clamped neighbour is itself a G,
  // which causes a minor miscolouring of those edge pixels.
  auto handle_border = [&](uint32_t y, uint32_t x) {
      auto p = [&](int32_t dy, int32_t dx) -> int {
          int32_t yy = std::clamp(static_cast<int32_t>(y) + dy, int32_t{0},
                              static_cast<int32_t>(height) - 1);
          int32_t xx = std::clamp(static_cast<int32_t>(x) + dx, int32_t{0},
                              static_cast<int32_t>(width) - 1);
          return src_mosaic[static_cast<size_t>(yy) * stride_in_pixels +
                   static_cast<size_t>(xx)];
        };
      const bool has_n = y > 0;
      const bool has_s = y + 1 < height;
      const bool has_w = x > 0;
      const bool has_e = x + 1 < width;

      auto avg_cardinal = [&]() -> int {
          int sum = 0;
          int count = 0;
          if (has_n) {
            sum += p(-1, 0);
            ++count;
          }
          if (has_s) {
            sum += p(1, 0);
            ++count;
          }
          if (has_w) {
            sum += p(0, -1);
            ++count;
          }
          if (has_e) {
            sum += p(0, 1);
            ++count;
          }
          return count ? (sum + count / 2) / count : 0;
        };
      auto avg_diagonal = [&]() -> int {
          int sum = 0;
          int count = 0;
          if (has_n && has_w) {
            sum += p(-1, -1);
            ++count;
          }
          if (has_n && has_e) {
            sum += p(-1, 1);
            ++count;
          }
          if (has_s && has_w) {
            sum += p(1, -1);
            ++count;
          }
          if (has_s && has_e) {
            sum += p(1, 1);
            ++count;
          }
          return count ? (sum + count / 2) / count : 0;
        };
      auto avg_horizontal = [&]() -> int {
          int sum = 0;
          int count = 0;
          if (has_w) {
            sum += p(0, -1);
            ++count;
          }
          if (has_e) {
            sum += p(0, 1);
            ++count;
          }
          return count ? (sum + count / 2) / count : 0;
        };
      auto avg_vertical = [&]() -> int {
          int sum = 0;
          int count = 0;
          if (has_n) {
            sum += p(-1, 0);
            ++count;
          }
          if (has_s) {
            sum += p(1, 0);
            ++count;
          }
          return count ? (sum + count / 2) / count : 0;
        };

      const int yp = static_cast<int>(y & 1u);
      const int xp = static_cast<int>(x & 1u);
      int r;
      int g;
      int b;

      if (yp == R_DY && xp == R_DX) {
        r = p(0, 0);
        g = avg_cardinal();
        b = avg_diagonal();
      } else if (yp == B_DY && xp == B_DX) {
        r = avg_diagonal();
        g = avg_cardinal();
        b = p(0, 0);
      } else {
        g = p(0, 0);
        if (yp == R_DY) {
          r = avg_horizontal();
          b = avg_vertical();
        } else {
          r = avg_vertical();
          b = avg_horizontal();
        }
      }
      write_pixel(y, x, r, g, b);
    };

  if (height < 3 || width < 3) {
    for (uint32_t y = 0; y < height; ++y) {
      for (uint32_t x = 0; x < width; ++x) {
        handle_border(y, x);
      }
    }
    return;
  }

  // Border passes.
  for (uint32_t x = 0; x < width; ++x) {
    handle_border(0, x);
  }
  for (uint32_t x = 0; x < width; ++x) {
    handle_border(height - 1, x);
  }
  for (uint32_t y = 1; y + 1 < height; ++y) {
    handle_border(y, 0);
  }
  for (uint32_t y = 1; y + 1 < height; ++y) {
    handle_border(y, width - 1);
  }

  // Interior
  for (uint32_t y = 1; y + 1 < height; ++y) {
    const T *row_m1 =
      src_mosaic + static_cast<size_t>(y - 1) * stride_in_pixels;
    const T *row_0 = src_mosaic + static_cast<size_t>(y) * stride_in_pixels;
    const T *row_p1 =
      src_mosaic + static_cast<size_t>(y + 1) * stride_in_pixels;
    const int yp = static_cast<int>(y & 1u);

    for (uint32_t x = 1; x + 1 < width; ++x) {
      const int p_nw = row_m1[x - 1];
      const int p_n = row_m1[x];
      const int p_ne = row_m1[x + 1];
      const int p_w = row_0[x - 1];
      const int p_c = row_0[x];
      const int p_e = row_0[x + 1];
      const int p_sw = row_p1[x - 1];
      const int p_s = row_p1[x];
      const int p_se = row_p1[x + 1];

      const int xp = static_cast<int>(x & 1u);
      int r;
      int g;
      int b;

      if (yp == R_DY && xp == R_DX) {
        r = p_c;
        g = (p_n + p_s + p_w + p_e + 2) / 4;
        b = (p_nw + p_ne + p_sw + p_se + 2) / 4;
      } else if (yp == B_DY && xp == B_DX) {
        r = (p_nw + p_ne + p_sw + p_se + 2) / 4;
        g = (p_n + p_s + p_w + p_e + 2) / 4;
        b = p_c;
      } else {
        g = p_c;
        if (yp == R_DY) {
          r = (p_w + p_e + 1) / 2;
          b = (p_n + p_s + 1) / 2;
        } else {
          r = (p_n + p_s + 1) / 2;
          b = (p_w + p_e + 1) / 2;
        }
      }
      write_pixel(y, x, r, g, b);
    }
  }
}

// Dispatch the layout to the templated implementation
template<typename T, typename Quantizer>
static void demosaicBayer(
  uint8_t *dst_rgb, const T *src_mosaic, BayerLayout layout,
  uint32_t height, uint32_t width, uint32_t stride_in_pixels,
  const Quantizer & quantize)
{
  static_assert(
    (std::is_same_v<T, uint8_t>) || (std::is_same_v<T, uint16_t>),
    "Bayer input must be uint8_t or uint16_t");
  static_assert(
    !(std::is_same_v<Quantizer, IdentityQuantizer>) || (std::is_same_v<T, uint8_t>),
    "IdentityQuantizer would truncate input wider than 8 bits");
  switch (layout) {
    case BayerLayout::RGGB:
      demosaicBayerImpl<T, 0, 0>(dst_rgb, src_mosaic, height, width,
                                 stride_in_pixels, quantize);
      return;
    case BayerLayout::BGGR:
      demosaicBayerImpl<T, 1, 1>(dst_rgb, src_mosaic, height, width,
                                 stride_in_pixels, quantize);
      return;
    case BayerLayout::GBRG:
      demosaicBayerImpl<T, 1, 0>(dst_rgb, src_mosaic, height, width,
                                 stride_in_pixels, quantize);
      return;
    case BayerLayout::GRBG:
      demosaicBayerImpl<T, 0, 1>(dst_rgb, src_mosaic, height, width,
                                 stride_in_pixels, quantize);
      return;
  }
}
}  // namespace displays
}  // namespace rviz_default_plugins
#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__IMAGE__DEMOSAIC_BAYER_HPP_
