/*
 * Copyright (c) 2012, Willow Garage, Inc.
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
 *     * Neither the name of the copyright holder nor the names of its contributors
 *       may be used to endorse or promote products derived from
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

#include "swatch.hpp"

#include <algorithm>

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# pragma GCC diagnostic ignored "-Wpedantic"
#endif

#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreRenderable.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTextureManager.h>
#include <OgreTechnique.h>
#include <OgreSharedPtr.h>

#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "rviz_rendering/custom_parameter_indices.hpp"

namespace rviz_default_plugins
{
namespace displays
{

// helper class to set alpha parameter on all renderables.
class AlphaSetter : public Ogre::Renderable::Visitor
{
public:
  explicit AlphaSetter(float alpha)
  : alpha_vec_(alpha, alpha, alpha, alpha)
  {}

  void
  visit(Ogre::Renderable * rend, Ogre::ushort lodIndex, bool isDebug, Ogre::Any * pAny) override
  {
    (void) lodIndex;
    (void) isDebug;
    (void) pAny;

    rend->setCustomParameter(RVIZ_RENDERING_ALPHA_PARAMETER, alpha_vec_);
  }

private:
  Ogre::Vector4 alpha_vec_;
};


Swatch::Swatch(
  Ogre::SceneManager * scene_manager,
  Ogre::SceneNode * parent_scene_node,
  size_t x, size_t y, size_t width, size_t height,
  float resolution, bool draw_under)
: scene_manager_(scene_manager), parent_scene_node_(parent_scene_node), manual_object_(nullptr),
  x_(x), y_(y), width_(width), height_(height)
{
  // Set up map material
  static int material_count = 0;
  std::stringstream ss;
  ss << "MapMaterial" << material_count++;
  material_ = Ogre::MaterialManager::getSingleton().getByName("rviz/Indexed8BitImage");
  material_ = material_->clone(ss.str());

  material_->setReceiveShadows(false);
  material_->getTechnique(0)->setLightingEnabled(false);
  material_->setDepthBias(-16.0f, 0.0f);
  material_->setCullingMode(Ogre::CULL_NONE);
  material_->setDepthWriteEnabled(false);

  static int map_count = 0;
  std::stringstream ss2;
  ss2 << "MapObject" << map_count++;
  manual_object_ = scene_manager_->createManualObject(ss2.str());

  static int node_count = 0;
  std::stringstream ss3;
  ss3 << "NodeObject" << node_count++;

  scene_node_ = parent_scene_node_->createChildSceneNode(ss3.str());
  scene_node_->attachObject(manual_object_);

  manual_object_->begin(
    material_->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST, "rviz_rendering");
  {
    // First triangle
    {
      // Bottom left
      manual_object_->position(0.0f, 0.0f, 0.0f);
      manual_object_->textureCoord(0.0f, 0.0f);
      manual_object_->normal(0.0f, 0.0f, 1.0f);

      // Top right
      manual_object_->position(1.0f, 1.0f, 0.0f);
      manual_object_->textureCoord(1.0f, 1.0f);
      manual_object_->normal(0.0f, 0.0f, 1.0f);

      // Top left
      manual_object_->position(0.0f, 1.0f, 0.0f);
      manual_object_->textureCoord(0.0f, 1.0f);
      manual_object_->normal(0.0f, 0.0f, 1.0f);
    }

    // Second triangle
    {
      // Bottom left
      manual_object_->position(0.0f, 0.0f, 0.0f);
      manual_object_->textureCoord(0.0f, 0.0f);
      manual_object_->normal(0.0f, 0.0f, 1.0f);

      // Bottom right
      manual_object_->position(1.0f, 0.0f, 0.0f);
      manual_object_->textureCoord(1.0f, 0.0f);
      manual_object_->normal(0.0f, 0.0f, 1.0f);

      // Top right
      manual_object_->position(1.0f, 1.0f, 0.0f);
      manual_object_->textureCoord(1.0f, 1.0f);
      manual_object_->normal(0.0f, 0.0f, 1.0f);
    }
  }
  manual_object_->end();

  scene_node_->setPosition(x * resolution, y * resolution, 0);
  scene_node_->setScale(width * resolution, height * resolution, 1.0);

  if (draw_under) {
    manual_object_->setRenderQueueGroup(Ogre::RENDER_QUEUE_4);
  }

  // don't show map until the plugin is actually enabled
  manual_object_->setVisible(false);
}

Swatch::~Swatch()
{
  scene_manager_->destroyManualObject(manual_object_);
}

void Swatch::updateAlpha(
  const Ogre::SceneBlendType sceneBlending, bool depthWrite, float alpha)
{
// TODO(holznera) figure out purpose of this.
//  Ogre::Pass * pass = material_->getTechnique(0)->getPass(0);
//  Ogre::TextureUnitState * tex_unit = NULL;

  AlphaSetter alpha_setter(alpha);

  material_->setSceneBlending(sceneBlending);
  material_->setDepthWriteEnabled(depthWrite);
  if (manual_object_) {
    manual_object_->visitRenderables(&alpha_setter);
  }
}

void Swatch::updateData(const nav_msgs::msg::OccupancyGrid & map)
{
  size_t pixels_size = width_ * height_;
  auto pixels = new unsigned char[pixels_size];
  memset(pixels, 255, pixels_size);
  unsigned char * ptr = pixels;
  size_t N = map.data.size();
  size_t fw = map.info.width;

  for (size_t yy = y_; yy < y_ + height_; yy++) {
    size_t index = yy * fw + x_;
    size_t pixels_to_copy = std::min(width_, N - index);
    memcpy(ptr, &map.data[index], pixels_to_copy);
    ptr += pixels_to_copy;
    if (index + pixels_to_copy >= N) {
      break;
    }
  }

  Ogre::DataStreamPtr pixel_stream;
  pixel_stream.reset(new Ogre::MemoryDataStream(pixels, pixels_size));

  if (texture_) {
    Ogre::TextureManager::getSingleton().remove(texture_);
    texture_.reset();
  }

  static int tex_count = 0;
  std::stringstream ss;
  ss << "MapTexture" << tex_count++;
  texture_ = Ogre::TextureManager::getSingleton().loadRawData(ss.str(),
      "rviz_rendering",
      pixel_stream, static_cast<uint8_t>(width_), static_cast<uint8_t>(height_),
      Ogre::PF_L8, Ogre::TEX_TYPE_2D, 0);

  delete[] pixels;
}

}  // namespace displays
}  // namespace rviz_default_plugins
