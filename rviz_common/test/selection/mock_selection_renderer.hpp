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

#ifndef SELECTION__MOCK_SELECTION_RENDERER_HPP_
#define SELECTION__MOCK_SELECTION_RENDERER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <OgreDataStream.h>
#include <OgreHardwarePixelBuffer.h>
#include <OgreCamera.h>

#include "rviz_common/selection/selection_manager.hpp"
#include "rviz_common/selection/selection_renderer.hpp"

#include "../display_context_fixture.hpp"

class VisibleObject
{
public:
  VisibleObject(int x, int y, rviz_common::DisplayContext * context)
  : x(x), y(y), handler_(std::make_shared<rviz_common::selection::SelectionHandler>(context))
  {}

  uint32_t getARGBColor()
  {
    return handler_->getHandle() & 0xFFFFFFFF;
  }

  rviz_common::selection::CollObjectHandle getHandle()
  {
    return handler_->getHandle();
  }

  int x;
  int y;

private:
  rviz_common::selection::SelectionHandlerPtr handler_;
};

class MockSelectionRenderer : public rviz_common::selection::SelectionRenderer
{
public:
  bool render(
    rviz_common::DisplayContext * vis_manager,
    Ogre::Camera * camera,
    rviz_common::selection::SelectionRectangle rectangle,
    rviz_common::selection::RenderTexture texture,
    Ogre::PixelBox & dst_box) override
  {
    (void) vis_manager;
    (void) camera;
    (void) texture;

    auto width = static_cast<uint32_t>(rectangle.x2_ - rectangle.x1_);
    auto height = static_cast<uint32_t>(rectangle.y2_ - rectangle.y1_);

    auto data = new uint8_t[Ogre::PixelUtil::getMemorySize(width, height, 1, Ogre::PF_A8R8G8B8)]();
    dst_box = Ogre::PixelBox(width, height, 1, Ogre::PF_A8R8G8B8, data);

    auto dstptr = static_cast<uint32_t *>(dst_box.data);
    for (auto & object : objects_) {
      int x = object.x - rectangle.x1_;
      int y = object.y - rectangle.y1_;
      if (x < rectangle.x2_ && y < rectangle.y2_) {
        dstptr[y * dst_box.rowPitch + x] = object.getARGBColor();
      }
    }
    return true;
  }

  void addVisibleObject(VisibleObject object)
  {
    objects_.emplace_back(object);
  }

  std::vector<VisibleObject> objects_;
};

#endif  // SELECTION__MOCK_SELECTION_RENDERER_HPP_
