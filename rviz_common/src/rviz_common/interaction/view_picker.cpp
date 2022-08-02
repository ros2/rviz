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

#include "rviz_common/interaction/view_picker.hpp"

#include <memory>
#include <string>
#include <vector>

#include <OgreCamera.h>
#include <OgreHardwarePixelBuffer.h>
#include <OgreRectangle2D.h>
#include <OgreRenderTexture.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTextureManager.h>
#include <OgreViewport.h>
#include <OgreVector.h>

#include "rviz_common/logging.hpp"

#include "rviz_common/display_context.hpp"
#include "rviz_common/interaction/handler_manager_iface.hpp"
#include "rviz_common/interaction/selection_renderer.hpp"
#include "rviz_common/render_panel.hpp"


namespace rviz_common
{
namespace interaction
{

ViewPicker::ViewPicker(
  DisplayContext * context, std::shared_ptr<SelectionRenderer> renderer)
: context_(context),
  renderer_(renderer)
{
  depth_pixel_box_.data = nullptr;
}

ViewPicker::ViewPicker(DisplayContext * context)
: ViewPicker(context, std::make_shared<rviz_common::interaction::SelectionRenderer>(context))
{}

ViewPicker::~ViewPicker()
{
  delete[] static_cast<uint8_t *>(depth_pixel_box_.data);
}

void ViewPicker::initialize()
{
  auto scene_manager = context_->getSceneManager();

  camera_ = scene_manager->createCamera("ViewPicker_camera");
  auto camera_node = scene_manager->getRootSceneNode()->createChildSceneNode();
  camera_node->attachObject(camera_);

  renderer_->initialize(camera_);

  handler_manager_ = context_->getHandlerManager();
}

bool ViewPicker::get3DPoint(
  RenderPanel * panel,
  int x,
  int y,
  Ogre::Vector3 & result_point)
{
  std::vector<Ogre::Vector3> result_points_temp;
  get3DPatch(panel, x, y, 1, 1, true, result_points_temp);
  if (result_points_temp.empty()) {
    return false;
  }
  result_point = result_points_temp[0];

  return true;
}

bool ViewPicker::get3DPatch(
  RenderPanel * panel,
  int x,
  int y,
  unsigned int width,
  unsigned int height,
  bool skip_missing,
  std::vector<Ogre::Vector3> & result_points)
{
  auto handler_lock = handler_manager_->lock();

  std::vector<float> depth_vector;

  getPatchDepthImage(panel, x, y, width, height, depth_vector);

  unsigned int pixel_counter = 0;
  Ogre::Matrix4 projection = camera_->getProjectionMatrix();
  float depth;

  for (unsigned int y_iter = 0; y_iter < height; ++y_iter) {
    for (unsigned int x_iter = 0; x_iter < width; ++x_iter) {
      depth = depth_vector[pixel_counter];

      // Deal with missing or invalid points
      if ((depth > camera_->getFarClipDistance()) || (depth == 0)) {
        ++pixel_counter;
        if (!skip_missing) {
          result_points.emplace_back(NAN, NAN, NAN);
        }
        continue;
      }

      Ogre::Vector3 result_point;
      // We want to shoot rays through the center of pixels, not the corners,
      // so add .5 pixels to the x and y coordinate to get to the center
      // instead of the top left of the pixel.
      Ogre::Real screenx = static_cast<float>(x_iter + .5) / static_cast<float>(width);
      Ogre::Real screeny = static_cast<float>(y_iter + .5) / static_cast<float>(height);
      result_point = projection[3][3] == 0.0 ?
        computeForPerspectiveProjection(depth, screenx, screeny) :
        computeForOrthogonalProjection(depth, screenx, screeny);

      result_points.push_back(result_point);
      ++pixel_counter;
    }
  }

  return !result_points.empty();
}

void ViewPicker::getPatchDepthImage(
  RenderPanel * panel, int x, int y, unsigned width,
  unsigned height, std::vector<float> & depth_vector)
{
  unsigned int num_pixels = width * height;
  depth_vector.reserve(num_pixels);

  setDepthTextureSize(width, height);

  render(
    panel->getRenderWindow(),
    SelectionRectangle(x, y, x + width, y + height),
    RenderTexture(
      depth_render_texture_, Dimensions(depth_texture_width_, depth_texture_height_), "Depth"),
    depth_pixel_box_);

  auto data_ptr = static_cast<uint8_t *>(depth_pixel_box_.data);

  // Assert that depth_pixel_box_ represents each depth pixel using 3 elements of type uint8_t,
  // and that the series of pixels in depth_pixel_box_.data is a contiguous array of sets of 3.
  // This ensures that the distance value at each pixel is composed using the correct indices.
  assert(Ogre::PF_R8G8B8 == depth_pixel_box_.format);

  for (uint32_t pixel = 0; pixel < num_pixels; ++pixel) {
    uint8_t a = data_ptr[3 * pixel];
    uint8_t b = data_ptr[3 * pixel + 1];
    uint8_t c = data_ptr[3 * pixel + 2];

    int int_depth = (c << 16) | (b << 8) | a;
    float normalized_depth = (static_cast<float>(int_depth)) / static_cast<float>(0xffffff);
    depth_vector.push_back(normalized_depth * camera_->getFarClipDistance());
  }
}

Ogre::Vector3
ViewPicker::computeForOrthogonalProjection(
  float depth, Ogre::Real screenx, Ogre::Real screeny) const
{
  Ogre::Ray ray;
  camera_->getCameraToViewportRay(screenx, screeny, &ray);

  return ray.getPoint(depth);
}

Ogre::Vector3
ViewPicker::computeForPerspectiveProjection(
  float depth, Ogre::Real screenx, Ogre::Real screeny) const
{
  // get world-space ray from camera & mouse coord
  Ogre::Ray vp_ray = camera_->getCameraToViewportRay(screenx, screeny);

  // transform ray direction back into camera coords
  Ogre::Vector3 dir_cam = camera_->getDerivedOrientation().Inverse() * vp_ray.getDirection();

  // normalize, so dir_cam.z == -depth
  dir_cam = dir_cam / dir_cam.z * depth * -1;

  // compute 3d point from camera origin and direction*/
  return camera_->getDerivedPosition() + camera_->getDerivedOrientation() * dir_cam;
}

void ViewPicker::setDepthTextureSize(unsigned width, unsigned height)
{
  // Cap and store requested texture sizecapTextureSize(width, height);
  capTextureSize(width, height);

  if (!depth_render_texture_.get() || depth_render_texture_->getWidth() != width ||
    depth_render_texture_->getHeight() != height)
  {
    std::string tex_name = "DepthTexture";
    if (depth_render_texture_.get()) {
      tex_name = depth_render_texture_->getName();

      // destroy old
      Ogre::TextureManager::getSingleton().remove(
        tex_name,
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    }

    depth_render_texture_ =
      Ogre::TextureManager::getSingleton().createManual(
      tex_name,
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
      Ogre::TEX_TYPE_2D, depth_texture_width_, depth_texture_height_, 0,
      Ogre::PF_R8G8B8,
      Ogre::TU_RENDERTARGET);

    depth_render_texture_->getBuffer()->getRenderTarget()->setAutoUpdated(false);
  }
}

void ViewPicker::capTextureSize(unsigned int & width, unsigned int & height)
{
  // It's probably an error if an invalid size is requested.
  if (width > 1024) {
    width = 1024;
    RVIZ_COMMON_LOG_ERROR_STREAM(
      "SelectionManager::setDepthTextureSize invalid width requested. "
      "Max Width: 1024 -- Width requested: " << width << ".  Capping Width at 1024.");
  }

  if (depth_texture_width_ != width) {
    depth_texture_width_ = width;
  }

  if (height > 1024) {
    height = 1024;
    RVIZ_COMMON_LOG_ERROR_STREAM(
      "SelectionManager::setDepthTextureSize invalid height requested. "
      "Max Height: 1024 -- Height requested: " << height << ".  Capping Height at 1024.");
  }

  if (depth_texture_height_ != height) {
    depth_texture_height_ = height;
  }
}

void ViewPicker::render(
  rviz_rendering::RenderWindow * window,
  const SelectionRectangle & selection_rectangle,
  const RenderTexture & render_texture,
  Ogre::PixelBox & dst_box)
{
  auto handler_lock = handler_manager_->lock();
  return renderer_->render(
    window, selection_rectangle,
    render_texture,
    handler_manager_->handlers(),
    dst_box);
}

}  // namespace interaction
}  // namespace rviz_common
