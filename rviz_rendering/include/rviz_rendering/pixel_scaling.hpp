// Copyright (c) 2026, Open Source Robotics Foundation, Inc.
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


#ifndef RVIZ_RENDERING__PIXEL_SCALING_HPP_
#define RVIZ_RENDERING__PIXEL_SCALING_HPP_

#include <QPoint>  // NOLINT: cpplint cannot handle include order here
#include <QtGlobal>  // NOLINT: cpplint cannot handle include order here

namespace rviz_rendering
{

/// Conversions between the two pixel units that meet at the Qt/Ogre boundary.
/**
 * Qt expresses window geometry and input events in logical pixels,
 * hiding the display scale factor from the application. Ogre has no notion
 * of a scale factor at all: it renders into a native surface and works purely in
 * device pixels.
 */
inline int
toDevicePixels(qreal logical, qreal pixel_ratio)
{
  return qRound(logical * pixel_ratio);
}

/// Convert a length in device pixels to logical pixels.
inline int
toLogicalPixels(qreal device, qreal pixel_ratio)
{
  return qRound(device / pixel_ratio);
}

/// Convert a point in logical pixels to device pixels.
inline QPoint
toDevicePixels(const QPoint & logical, qreal pixel_ratio)
{
  return QPoint(
    toDevicePixels(logical.x(), pixel_ratio),
    toDevicePixels(logical.y(), pixel_ratio));
}

/// Convert a point in device pixels to logical pixels.
inline QPoint
toLogicalPixels(const QPoint & device, qreal pixel_ratio)
{
  return QPoint(
    toLogicalPixels(device.x(), pixel_ratio),
    toLogicalPixels(device.y(), pixel_ratio));
}

}  // namespace rviz_rendering

#endif  // RVIZ_RENDERING__PIXEL_SCALING_HPP_
