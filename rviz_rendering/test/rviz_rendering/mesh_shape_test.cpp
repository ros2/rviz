/*
 * Copyright (c) 2023, Open Source Robotics Foundation, Inc.
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

#include <gmock/gmock.h>

#include <OgreMesh.h>
#include <OgreSubMesh.h>
#include <OgreRoot.h>
#include <OgreSceneNode.h>

#include <memory>

#include "./ogre_testing_environment.hpp"
#include "./scene_graph_introspection.hpp"
#include "rviz_rendering/objects/mesh_shape.hpp"

using namespace ::testing;  // NOLINT

class MeshShapeTestFixture : public ::testing::Test
{
protected:
  void SetUp()
  {
    testing_environment_ = std::make_shared<rviz_rendering::OgreTestingEnvironment>();
    testing_environment_->setUpOgreTestEnvironment();
  }

  std::shared_ptr<rviz_rendering::OgreTestingEnvironment> testing_environment_;
};

TEST_F(MeshShapeTestFixture, mesh_shape_test) {
  auto scene_manager = Ogre::Root::getSingletonPtr()->createSceneManager();
  auto root_node = scene_manager->getRootSceneNode();

  auto mesh_shape_visual = std::make_shared<rviz_rendering::MeshShape>(
    scene_manager, root_node);
  ASSERT_NE(nullptr, mesh_shape_visual);
  ASSERT_NE(nullptr, mesh_shape_visual->getManualObject());

  struct Triangle
  {
    unsigned v1, v2, v3;  // index for the 3 vertices that make up a triangle
  };
  std::vector<Triangle> triangles = {
    {0, 1, 2},
    {2, 3, 4}
  };
  std::vector<Ogre::Vector3> vertices = {
    {-0.5f, 0.5f, -0.5f},
    {-0.5f, -0.5f, -0.5f},
    {0.5f, 0.5f, -0.5f},
    {-0.5f, -0.5f, -0.5f},
    {0.5f, -0.5f, -0.5f}
  };

  std::vector<Ogre::Vector3> normals = {
    {0.0f, 0.0f, 1.0f},
    {0.0f, 0.0f, 1.0f}
  };

  Ogre::ManualObject * object = mesh_shape_visual->getManualObject();

  EXPECT_THROW(object->convertToMesh("mesh"), Ogre::InvalidParametersException);

  mesh_shape_visual->estimateVertexCount(vertices.size());
  mesh_shape_visual->beginTriangles();
  for (std::size_t i = 0; i < vertices.size(); ++i) {
    mesh_shape_visual->addVertex(vertices[i], normals[i]);
  }
  for (std::size_t i = 0; i < triangles.size(); ++i) {
    mesh_shape_visual->addTriangle(triangles[i].v1, triangles[i].v2, triangles[i].v3);
  }
  mesh_shape_visual->endTriangles();

  object = mesh_shape_visual->getManualObject();

  EXPECT_EQ(
    vertices.size(),
    object->convertToMesh("mesh")->getSubMesh(0)->vertexData->vertexCount);

  mesh_shape_visual->clear();

  EXPECT_THROW(object->convertToMesh("mesh"), Ogre::InvalidParametersException);
}
