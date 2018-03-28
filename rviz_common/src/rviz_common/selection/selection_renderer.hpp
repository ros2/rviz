/*
 * Copyright (c) 2017, Open Source Robotics Foundation, Inc.
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
 *     * Neither the name of the copyright holder nor the names of its
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

#ifndef RVIZ_COMMON__SELECTION__SELECTION_RENDERER_HPP_
#define RVIZ_COMMON__SELECTION__SELECTION_RENDERER_HPP_

#include <map>
#include <memory>
#include <string>

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

#include <OgreMaterialManager.h>
#include <OgreRenderQueueListener.h>

#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <src/rviz_common/visualization_manager.hpp>

namespace rviz_common
{
namespace selection
{

class SelectionRenderer
  : public Ogre::MaterialManager::Listener, public Ogre::RenderQueueListener
{
public:
  SelectionRenderer();
  ~SelectionRenderer() override = default;

  virtual void initialize();

  virtual
  bool render(
    rviz_common::DisplayContext * vis_manager,
    Ogre::Camera * camera,
    Ogre::Viewport * viewport,
    Ogre::TexturePtr tex,
    int x1,
    int y1,
    int x2,
    int y2,
    Ogre::PixelBox & dst_box,
    std::string material_scheme,
    unsigned texture_width,
    unsigned texture_height);

  /// Implementation for Ogre::RenderQueueListener.
  void renderQueueStarted(
    uint8_t queueGroupId,
    const std::string & invocation,
    bool & skipThisInvocation) override;

  /// Implementation for Ogre::MaterialManager::Listener
  /// If a material does not support the picking scheme, paint it black.
  Ogre::Technique * handleSchemeNotFound(
    unsigned short scheme_index,  // NOLINT: Ogre decides the use of unsigned short
    const Ogre::String & scheme_name,
    Ogre::Material * original_material,
    unsigned short lod_index,  // NOLINT: Ogre decides the use of unsigned short
    const Ogre::Renderable * rend) override;

  void setDebugMode(bool debug);

private:
  void publishDebugImage(const Ogre::PixelBox & pixel_box, const std::string & label);

  Ogre::MaterialPtr fallback_pick_material_;
  Ogre::Technique * fallback_pick_technique_;
  Ogre::Technique * fallback_black_technique_;
  Ogre::Technique * fallback_depth_technique_;
  Ogre::Technique * fallback_pick_cull_technique_;
  Ogre::Technique * fallback_black_cull_technique_;
  Ogre::Technique * fallback_depth_cull_technique_;

  bool debug_mode_;
  rclcpp::Node debug_publisher_node_;
  typedef std::map<std::string, std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>>>
    PublisherMap;
  PublisherMap debug_publishers_;
};

}  // namespace selection
}  // namespace rviz_common

#endif  // RVIZ_COMMON__SELECTION__SELECTION_RENDERER_HPP_
