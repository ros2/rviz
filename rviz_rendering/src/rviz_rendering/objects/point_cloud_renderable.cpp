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

#include "rviz_rendering/objects/point_cloud_renderable.hpp"

#include <algorithm>

#include <OgreCamera.h>

#include "rviz_rendering/objects/point_cloud.hpp"

namespace rviz_rendering
{

PointCloudRenderable::PointCloudRenderable(
  PointCloud * parent, int num_points, bool
  use_tex_coords, Ogre::RenderOperation::OperationType operationType)
: parent_(parent)
{
  initializeRenderOperation(operationType);
  specifyBufferContent(use_tex_coords);
  createAndBindBuffer(num_points);
}

PointCloudRenderable::~PointCloudRenderable()
{
  delete mRenderOp.vertexData;
  delete mRenderOp.indexData;
}

Ogre::HardwareVertexBufferSharedPtr PointCloudRenderable::getBuffer()
{
  return mRenderOp.vertexData->vertexBufferBinding->getBuffer(0);
}

void PointCloudRenderable::_notifyCurrentCamera(Ogre::Camera * camera)
{
  Ogre::SimpleRenderable::_notifyCurrentCamera(camera);
}

Ogre::Real PointCloudRenderable::getBoundingRadius() const
{
  return Ogre::Math::Sqrt(
    std::max(
      mBox.getMaximum().squaredLength(),
      mBox.getMinimum().squaredLength()));
}

Ogre::Real PointCloudRenderable::getSquaredViewDepth(const Ogre::Camera * cam) const
{
  Ogre::Vector3 vMin, vMax, vMid, vDist;
  vMin = mBox.getMinimum();
  vMax = mBox.getMaximum();
  vMid = ((vMax - vMin) * 0.5) + vMin;
  vDist = cam->getDerivedPosition() - vMid;

  return vDist.squaredLength();
}

void PointCloudRenderable::getWorldTransforms(Ogre::Matrix4 * xform) const
{
  parent_->getWorldTransforms(xform);
}

const Ogre::LightList & PointCloudRenderable::getLights() const
{
  return parent_->queryLights();
}

void PointCloudRenderable::initializeRenderOperation(
  Ogre::RenderOperation::OperationType operation_type)
{
  mRenderOp.operationType = operation_type;
  mRenderOp.useIndexes = false;
  mRenderOp.vertexData = new Ogre::VertexData;
  mRenderOp.vertexData->vertexStart = 0;
  mRenderOp.vertexData->vertexCount = 0;
}

void PointCloudRenderable::specifyBufferContent(bool use_tex_coords)
{
  Ogre::VertexDeclaration * declaration = mRenderOp.vertexData->vertexDeclaration;
  size_t offset = 0;

  declaration->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
  offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);

  if (use_tex_coords) {
    declaration->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_TEXTURE_COORDINATES, 0);
    offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);
  }

  declaration->addElement(0, offset, Ogre::VET_COLOUR, Ogre::VES_DIFFUSE);
}

void PointCloudRenderable::createAndBindBuffer(int num_points)
{
  Ogre::HardwareVertexBufferSharedPtr vertexBuffer =
    Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
    mRenderOp.vertexData->vertexDeclaration->getVertexSize(0),
    num_points,
    Ogre::HardwareBuffer::HBU_DYNAMIC);

  mRenderOp.vertexData->vertexBufferBinding->setBinding(0, vertexBuffer);
}

}  // namespace rviz_rendering
