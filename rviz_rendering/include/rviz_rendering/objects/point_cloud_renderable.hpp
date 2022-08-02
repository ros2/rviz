/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * Copyright (c) 2017, Bosch Software Innovations GmbH.
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

#ifndef RVIZ_RENDERING__OBJECTS__POINT_CLOUD_RENDERABLE_HPP_
#define RVIZ_RENDERING__OBJECTS__POINT_CLOUD_RENDERABLE_HPP_

#include <deque>
#include <memory>

#include <OgreSimpleRenderable.h>
#include <OgreString.h>
#include <OgreAxisAlignedBox.h>
#include <OgreVector.h>
#include <OgreMaterial.h>
#include <OgreColourValue.h>
#include <OgreRoot.h>
#include <OgreHardwareBufferManager.h>
#include <OgreSharedPtr.h>

#include "rviz_rendering/visibility_control.hpp"

namespace Ogre
{
class SceneManager;
class ManualObject;
class SceneNode;
class RenderQueue;
class Camera;
class RenderSystem;
class Matrix4;
}

namespace rviz_rendering
{

class PointCloud;

class PointCloudRenderable : public Ogre::SimpleRenderable
{
public:
  RVIZ_RENDERING_PUBLIC
  PointCloudRenderable(
    PointCloud * parent, int num_points, bool use_tex_coords,
    Ogre::RenderOperation::OperationType operationType);

  RVIZ_RENDERING_PUBLIC
  virtual ~PointCloudRenderable();

  RVIZ_RENDERING_PUBLIC
  Ogre::RenderOperation * getRenderOperation() {return &mRenderOp;}

  // Avoid hidding parent class overload.
  using Ogre::SimpleRenderable::getRenderOperation;

  RVIZ_RENDERING_PUBLIC
  Ogre::HardwareVertexBufferSharedPtr getBuffer();

  RVIZ_RENDERING_PUBLIC
  Ogre::Real getBoundingRadius() const override;

  RVIZ_RENDERING_PUBLIC
  Ogre::Real getSquaredViewDepth(const Ogre::Camera * cam) const override;

  RVIZ_RENDERING_PUBLIC
  void _notifyCurrentCamera(Ogre::Camera * camera) override;

  RVIZ_RENDERING_PUBLIC
  uint16_t getNumWorldTransforms() const override {return 1;}

  RVIZ_RENDERING_PUBLIC
  void getWorldTransforms(Ogre::Matrix4 * xform) const override;

  RVIZ_RENDERING_PUBLIC
  const Ogre::LightList & getLights() const override;

private:
  RVIZ_RENDERING_PUBLIC
  void initializeRenderOperation(Ogre::RenderOperation::OperationType operation_type);

  RVIZ_RENDERING_PUBLIC
  void specifyBufferContent(bool use_tex_coords);

  RVIZ_RENDERING_PUBLIC
  void createAndBindBuffer(int num_points);

  PointCloud * parent_;
};
typedef std::shared_ptr<PointCloudRenderable> PointCloudRenderablePtr;
typedef std::deque<PointCloudRenderablePtr> PointCloudRenderableQueue;

}  // namespace rviz_rendering

#endif  // RVIZ_RENDERING__OBJECTS__POINT_CLOUD_RENDERABLE_HPP_
