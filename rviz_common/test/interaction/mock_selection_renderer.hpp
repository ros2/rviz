/*
 * Copyright (c) 2018, Bosch Software Innovations GmbH.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef INTERACTION__MOCK_SELECTION_RENDERER_HPP_
#define INTERACTION__MOCK_SELECTION_RENDERER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <OgreDataStream.h>
#include <OgreHardwarePixelBuffer.h>
#include <OgreCamera.h>

#include "rviz_common/interaction/selection_manager.hpp"
#include "rviz_common/interaction/selection_renderer.hpp"

#include "../display_context_fixture.hpp"

class VisibleObject
{
public:
  VisibleObject(int x, int y, rviz_common::DisplayContext * context)
  : x(x), y(y), handler_(
      rviz_common::interaction::createSelectionHandler
      <rviz_common::interaction::SelectionHandler>(context))
  {}

  uint32_t getARGBColor()
  {
    return handler_->getHandle() & 0xFFFFFFFF;
  }

  rviz_common::interaction::CollObjectHandle getHandle()
  {
    return handler_->getHandle();
  }

  int x;
  int y;

private:
  rviz_common::interaction::SelectionHandlerPtr handler_;
};

class MockSelectionRenderer : public rviz_common::interaction::SelectionRenderer
{
public:
  explicit MockSelectionRenderer(rviz_common::DisplayContext * context)
  : SelectionRenderer(context) {}

  void render(
    rviz_rendering::RenderWindow * window,
    rviz_common::interaction::SelectionRectangle rectangle,
    rviz_common::interaction::RenderTexture texture,
    rviz_common::interaction::HandlerRange handlers,
    Ogre::PixelBox & dst_box) override
  {
    (void) window;
    (void) texture;
    (void) handlers;

    auto width = static_cast<uint32_t>(rectangle.x2 - rectangle.x1);
    auto height = static_cast<uint32_t>(rectangle.y2 - rectangle.y1);

    auto data = new uint8_t[Ogre::PixelUtil::getMemorySize(width, height, 1, Ogre::PF_A8R8G8B8)]();
    delete[] static_cast<uint8_t *>(dst_box.data);
    dst_box = Ogre::PixelBox(width, height, 1, Ogre::PF_A8R8G8B8, data);

    // The data is stored in uchars where four contiguous uchars correspond to one pixel (each
    // one channel alpha, red, green, blue). Since the SelectionHandler uses one uint32_t for
    // each pixel everywhere, we reinterpret the data here to make the code more readable.
    auto dstptr = reinterpret_cast<uint32_t *>(dst_box.data);
    for (auto & object : objects_) {
      int x = object.x - rectangle.x1;
      int y = object.y - rectangle.y1;
      if (x < rectangle.x2 && y < rectangle.y2 && x >= 0 && y >= 0) {
        dstptr[y * dst_box.rowPitch + x] = object.getARGBColor();
      }
    }
  }

  void addVisibleObject(VisibleObject object)
  {
    objects_.emplace_back(object);
  }

  std::vector<VisibleObject> objects_;
};

#endif  // INTERACTION__MOCK_SELECTION_RENDERER_HPP_
