/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * Copyright (c) 2017, Open Source Robotics Foundation, Inc.
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
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
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

#ifndef RVIZ_COMMON__INTERACTION__VIEW_PICKER_HPP_
#define RVIZ_COMMON__INTERACTION__VIEW_PICKER_HPP_

#include "rviz_common/interaction/view_picker_iface.hpp"

#include <memory>
#include <vector>

#include <OgreMaterialManager.h>
#include <OgreRenderQueueListener.h>
#include <OgreVector.h>

#include <QObject>  // NOLINT: cpplint is unable to handle the include order here

#include "rviz_common/interaction/forwards.hpp"
#include "rviz_common/interaction/selection_handler.hpp"
#include "rviz_common/visibility_control.hpp"

namespace Ogre
{

class Rectangle2D;

}  // namespace Ogre

namespace rviz_rendering
{

class RenderWindow;

}  // namespace rviz_rendering

namespace rviz_common
{

class DisplayContext;

namespace interaction
{

class HandlerManagerIface;
class SelectionRenderer;
struct SelectionRectangle;
struct RenderTexture;

class RVIZ_COMMON_PUBLIC ViewPicker
  : public ViewPickerIface
{
public:
  ViewPicker(DisplayContext * manager, std::shared_ptr<SelectionRenderer> renderer);

  explicit ViewPicker(DisplayContext * manager);

  ~ViewPicker() override;

  void
  initialize() override;

  /// Return true if the point at x, y in the viewport is showing an object, false otherwise.
  /**
   * If it is showing an object, result will be changed to contain the 3D point
   * corresponding to it.
   */
  bool get3DPoint(
    RenderPanel * panel,
    int x,
    int y,
    Ogre::Vector3 & result_point) override;

  /// Gets the 3D points in a box around a point in a view port.
  /**
   * \param[in] viewport Rendering area clicked on.
   * \param[in] x x coordinate of upper-left corner of box.
   * \param[in] y y coordinate of upper-left corner of box.
   * \param[in] width The width of the rendered box in pixels.
   * \param[in] height The height of the rendered box in pixels.
   * \param[in] skip_missing Whether to skip non-existing points or insert
   *   NaNs for them
   * \param[out] result_points The vector of output points.
   *
   * \returns True if any valid point is rendered in the box. NaN points count,
   *   so if skip_missing is false, this will always return true if
   *   width and height are > 0.
   */
  bool get3DPatch(
    RenderPanel * panel,
    int x,
    int y,
    unsigned width,
    unsigned height,
    bool skip_missing,
    std::vector<Ogre::Vector3> & result_points) override;

private:
  void setDepthTextureSize(unsigned width, unsigned height);

  void capTextureSize(unsigned int & width, unsigned int & height);

  /**
   * \param[in] panel Rendering area clicked on.
   * \param[in] x x coordinate of upper-left corner of box.
   * \param[in] y y coordinate of upper-left corner of box.
   * \param[in] width The width of the rendered box in pixels.
   * \param[in] height The height of the rendered box in pixels.
   * \param[out] depth_vector The vector of depth values.
   *
   * \returns True if rendering operation to render depth data to the depth
   *   texture buffer succeeds.
   *   Failure likely indicates a pretty serious problem.
   */
  void getPatchDepthImage(
    RenderPanel * panel,
    int x,
    int y,
    unsigned width,
    unsigned height,
    std::vector<float> & depth_vector);

  Ogre::Vector3
  computeForOrthogonalProjection(float depth, Ogre::Real screenx, Ogre::Real screeny) const;

  Ogre::Vector3 computeForPerspectiveProjection(
    float depth, Ogre::Real screenx, Ogre::Real screeny) const;
  /// Renders a depth image in a box around a point in a view port.
  /// Internal render function to render to a texture and read the pixels back out.
  void render(
    rviz_rendering::RenderWindow * window,
    const SelectionRectangle & selection_rectangle,
    const RenderTexture & render_texture,
    Ogre::PixelBox & dst_box);

  DisplayContext * context_;
  std::shared_ptr<HandlerManagerIface> handler_manager_;

  // Graphics card -based depth finding of clicked points.
  Ogre::TexturePtr depth_render_texture_;
  uint32_t depth_texture_width_, depth_texture_height_;

  Ogre::PixelBox depth_pixel_box_;

  Ogre::Camera * camera_;

  std::shared_ptr<rviz_common::interaction::SelectionRenderer> renderer_;
};

}  // namespace interaction
}  // namespace rviz_common

#endif  // RVIZ_COMMON__INTERACTION__VIEW_PICKER_HPP_
