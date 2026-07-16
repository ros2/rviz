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

#include "rviz_common/interaction/forwards.hpp"

#include <cstdint>

#include <OgrePixelFormat.h>
#include <OgreColourValue.h>

#include "rviz_common/logging.hpp"

namespace rviz_common
{
namespace interaction
{

uint32_t colorToHandle(Ogre::PixelFormat fmt, uint32_t col)
{
  uint32_t handle = 0;
  if (fmt == Ogre::PF_A8R8G8B8 || fmt == Ogre::PF_X8R8G8B8) {
    handle = col & 0x00ffffff;
  } else if (fmt == Ogre::PF_R8G8B8A8) {
    handle = col >> 8;
  } else {
    RVIZ_COMMON_LOG_DEBUG_STREAM("Incompatible pixel format [" << fmt << "]");
  }

  return handle;
}

CollObjectHandle colorToHandle(const Ogre::ColourValue & color)
{
  return
    (static_cast<int>(color.r * 255) << 16) |
    (static_cast<int>(color.g * 255) << 8) |
    static_cast<int>(color.b * 255);
}

}  // namespace interaction
}  // namespace rviz_common
