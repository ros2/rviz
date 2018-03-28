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

#include <OgreDataStream.h>
#include <OgreHardwarePixelBuffer.h>
#include <OgreCamera.h>

#include "rviz_common/selection/selection_manager.hpp"
#include "../../src/rviz_common/selection/selection_renderer.hpp"

#include "../display_context_fixture.hpp"

// TODO(Martin-Idel-SI): It might be possible to convert this to a regular mock when refactoring
// the rendering function to not work with output parameters.
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
    (void) rectangle;
    (void) texture;

    if (!rendering_) {
      return false;
    }

    Ogre::Image image = loadImage(file_name_);

    auto pixel_box = image.getPixelBox();

    convertFormat(pixel_box, dst_box);

    // TODO(Martin-Idel-SI): the real render function crops the image to the size given by the
    // SelectionRectangle.
    // If we don't crop, everything is fine, but maybe there is some functionality that needs
    // testing. If not, remove after testing.
    return true;
  }

  void setFileName(std::string file_name)
  {
    file_name_ = file_name;
  }

  void isRendering(bool rendering)
  {
    rendering_ = rendering;
  }

private:
  // Ogre's save format PF_A8B8G8R8 is not supported
  void convertFormat(const Ogre::PixelBox & pixel_box, Ogre::PixelBox & dst_box) const
  {
    delete[] reinterpret_cast<uint8_t *>(dst_box.data);
    auto size = static_cast<int>(Ogre::PixelUtil::getMemorySize(
        pixel_box.getWidth(), pixel_box.getHeight(), 1, pixel_box.format));
    auto data = new uint8_t[size];
    dst_box = Ogre::PixelBox(
      pixel_box.getWidth(), pixel_box.getHeight(), 1, Ogre::PF_A8R8G8B8, data);

    // Convert pixels manually since Ogre::bulkPixelConvert() didn't work
    uint32_t * srcptr = static_cast<uint32_t *>(pixel_box.data);
    uint32_t * dstptr = static_cast<uint32_t *>(dst_box.data);

    for (size_t y = pixel_box.top; y < pixel_box.bottom; ++y) {
      for (size_t x = pixel_box.left; x < pixel_box.right; x++) {
        auto inp = srcptr[x];
        dstptr[x] = ((inp & 0x000000FF) << 16) | (inp & 0xFF00FF00) | ((inp & 0x00FF0000) >> 16);
      }
      srcptr += pixel_box.rowPitch;
      dstptr += dst_box.rowPitch;
    }
  }

  Ogre::Image loadImage(const std::string & file_name) const
  {
    std::string source_directory_path(_SRC_DIR_PATH);
    std::ifstream image_file(
      Ogre::String(source_directory_path + "/test_resources/" + file_name).c_str(),
      std::ios_base::in | std::ios_base::binary);

    Ogre::DataStreamPtr image_data = Ogre::DataStreamPtr(
      OGRE_NEW Ogre::FileStreamDataStream(&image_file, false));

    Ogre::Image image = Ogre::Image();
    return image.load(image_data, "png");
  }

  bool rendering_ = true;
  std::string file_name_ = "";
};

#endif  // SELECTION__MOCK_SELECTION_RENDERER_HPP_
