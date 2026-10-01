// Copyright (c) 2026, John C. Furey
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

#include <QSize>  // NOLINT

#include "rviz_rendering/pixel_scaling.hpp"

TEST(PixelScaling, rounds_odd_surface_dimensions_at_fractional_scales)
{
  struct Sample
  {
    QSize logical_size;
    qreal ratio;
    QSize device_size;
  };
  const Sample samples[] = {
    {QSize(321, 243), 1.0, QSize(321, 243)},
    {QSize(321, 243), 1.25, QSize(401, 304)},
    {QSize(321, 243), 1.5, QSize(482, 365)},
    {QSize(321, 243), 2.0, QSize(642, 486)},
    {QSize(503, 301), 1.25, QSize(629, 376)},
    {QSize(503, 301), 1.5, QSize(755, 452)},
    {QSize(257, 199), 1.25, QSize(321, 249)},
    {QSize(257, 199), 1.5, QSize(386, 299)},
  };
  for (const auto & sample : samples) {
    SCOPED_TRACE(
      testing::Message() << sample.logical_size.width() << "x" << sample.logical_size.height()
                         << " at " << sample.ratio);
    const QSize device_size(
      rviz_rendering::toDevicePixels(sample.logical_size.width(), sample.ratio),
      rviz_rendering::toDevicePixels(sample.logical_size.height(), sample.ratio));
    EXPECT_EQ(device_size, sample.device_size);
    EXPECT_EQ(device_size, sample.logical_size * sample.ratio);
  }
}
