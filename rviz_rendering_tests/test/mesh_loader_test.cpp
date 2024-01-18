/*
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
 *     * Neither the name of the copyright holder, nor the names of its
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

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <memory>
#include <string>
#include <vector>

#include <OgreRoot.h>
#include <OgreSubMesh.h>
#include <OgreMaterialManager.h>
#include "resource_retriever/retriever.hpp"

#include "ogre_testing_environment.hpp"
#include "rviz_rendering/mesh_loader.hpp"

using namespace ::testing;  // NOLINT

class MeshLoaderTestFixture : public ::testing::Test
{
protected:
  void SetUp()
  {
    testing_environment_ = std::make_shared<rviz_rendering_tests::OgreTestingEnvironment>();
    testing_environment_->setUpOgreTestEnvironment();
  }

  std::shared_ptr<rviz_rendering_tests::OgreTestingEnvironment> testing_environment_;
};

void assertVector3Equality(Ogre::Vector3 actual, Ogre::Vector3 expected)
{
  ASSERT_NEAR(actual.x, expected.x, 0.0001);
  ASSERT_NEAR(actual.y, expected.y, 0.0001);
  ASSERT_NEAR(actual.z, expected.z, 0.0001);
}
void assertBoundingBoxEquality(Ogre::AxisAlignedBox actual, Ogre::AxisAlignedBox expected)
{
  assertVector3Equality(actual.getMaximum(), expected.getMaximum());
  assertVector3Equality(actual.getMinimum(), expected.getMinimum());
}

TEST_F(MeshLoaderTestFixture, throws_reasonable_exception_for_missing_files) {
  std::string mesh_path = "package://rviz_rendering/ogre_media/MISSING.mesh";
  testing::internal::CaptureStderr();

  auto mesh = rviz_rendering::loadMeshFromResource(mesh_path);

  std::string output = testing::internal::GetCapturedStderr();
  ASSERT_THAT(output, HasSubstr("Error retrieving file"));
}

TEST_F(MeshLoaderTestFixture, can_load_ogre_mesh_files) {
  std::string mesh_path = "package://rviz_rendering/ogre_media/models/rviz_sphere.mesh";

  auto mesh = rviz_rendering::loadMeshFromResource(mesh_path);

  ASSERT_TRUE(mesh->isManuallyLoaded());
  ASSERT_EQ(mesh_path, mesh->getName());
}

TEST_F(MeshLoaderTestFixture, can_load_stl_files) {
  std::string mesh_path = "package://rviz_rendering_tests/test_meshes/F2.stl";

  auto mesh = rviz_rendering::loadMeshFromResource(mesh_path);

  float expected_bound_radius = 34.920441f;
  size_t expected_vertex_count = 35532;
  size_t actual_vertex_count = 0;
  for (size_t i = 0; i < mesh->getNumSubMeshes(); ++i) {
    actual_vertex_count += mesh->getSubMesh(i)->vertexData->vertexCount;
  }
  ASSERT_TRUE(mesh->isManuallyLoaded());
  ASSERT_EQ(mesh_path, mesh->getName());
  ASSERT_FLOAT_EQ(expected_bound_radius, mesh->getBoundingSphereRadius());
  ASSERT_EQ(expected_vertex_count, actual_vertex_count);
}

TEST_F(MeshLoaderTestFixture, loading_invalid_short_stl_files_fail) {
  /// Load an invalid STL binary file (size < 84 bytes).
  std::string mesh_path = "package://rviz_rendering_tests/test_meshes/invalid_short.stl";

  ASSERT_FALSE(rviz_rendering::loadMeshFromResource(mesh_path));
}

TEST_F(MeshLoaderTestFixture, loading_invalid_stl_files_fail) {
  /// Load an invalid STL binary file (size does not match the expected size).
  std::string mesh_path = "package://rviz_rendering_tests/test_meshes/invalid.stl";

  ASSERT_FALSE(rviz_rendering::loadMeshFromResource(mesh_path));
}

TEST_F(MeshLoaderTestFixture, loading_invalid_ascii_stl_file) {
  /// Load an invalid STL binary file (size does not match the expected size).
  std::string mesh_path = "package://rviz_rendering_tests/test_meshes/invalid_ascii.stl";

  ASSERT_FALSE(rviz_rendering::loadMeshFromResource(mesh_path));
}

TEST_F(MeshLoaderTestFixture, loading_invalid_stl_files_should_fail) {
  /// Load an invalid STL binary file (size does not match the expected size,
  /// but does if incorrectly read as an 16-bit uint)
  std::string mesh_path =
    "package://rviz_rendering_tests/test_meshes/16bit_vs_32bit_should_fail.stl";

  ASSERT_FALSE(rviz_rendering::loadMeshFromResource(mesh_path));
}

TEST_F(MeshLoaderTestFixture, loading_almost_invalid_stl_files_should_fail) {
  /// Load a "potentially" valid STL binary file with bigger size than the
  /// expected. The file will not load.
  std::string mesh_path = "package://rviz_rendering_tests/test_meshes/invalid_extra.stl";

  EXPECT_FALSE(rviz_rendering::loadMeshFromResource(mesh_path));
}

TEST_F(MeshLoaderTestFixture, loading_stl_mesh_twice_should_not_fail) {
  std::string mesh_path = "package://rviz_rendering_tests/test_meshes/F2.stl";

  ASSERT_TRUE(rviz_rendering::loadMeshFromResource(mesh_path));
  ASSERT_TRUE(rviz_rendering::loadMeshFromResource(mesh_path));
}

TEST_F(MeshLoaderTestFixture, can_load_assimp_mesh_files) {
  std::string mesh_path = "package://rviz_rendering_tests/test_meshes/pr2-base.dae";

  auto mesh = rviz_rendering::loadMeshFromResource(mesh_path);

  size_t expected_vertex_number = 3600;
  size_t actual_vertex_number = mesh->getSubMesh(0)->vertexData->vertexCount;
  float expected_bounding_radius = 0.754754f;
  Ogre::AxisAlignedBox expected_bounding_box = Ogre::AxisAlignedBox(
    Ogre::Vector3(-0.342375f, -0.340043f, -0.00656401f),
    Ogre::Vector3(0.340425f, 0.340043f, 0.662748f));
  ASSERT_TRUE(mesh->isManuallyLoaded());
  ASSERT_EQ(mesh_path, mesh->getName());
  ASSERT_EQ(expected_vertex_number, actual_vertex_number);
  ASSERT_FLOAT_EQ(expected_bounding_radius, mesh->getBoundingSphereRadius());
  assertBoundingBoxEquality(expected_bounding_box, mesh->getBounds());
}

TEST_F(MeshLoaderTestFixture, assimp_loader_reads_size_correctly) {
  std::string mesh_path = "package://rviz_rendering_tests/test_meshes/pr2-base_large.dae";

  auto mesh = rviz_rendering::loadMeshFromResource(mesh_path);

  // The pr2-base_large.dae mesh should be 6 times larger than pr2-base.dae.
  float expected_bounding_radius = 6 * 0.754754f;
  Ogre::AxisAlignedBox expected_bounding_box = Ogre::AxisAlignedBox(
    Ogre::Vector3(6 * -0.342375f, 6 * -0.340043f, 6 * -0.00656401f),
    Ogre::Vector3(6 * 0.340425f, 6 * 0.340043f, 6 * 0.662748f));
  ASSERT_TRUE(mesh->isManuallyLoaded());
  ASSERT_EQ(mesh_path, mesh->getName());
  ASSERT_FLOAT_EQ(expected_bounding_radius, mesh->getBoundingSphereRadius());
  assertBoundingBoxEquality(expected_bounding_box, mesh->getBounds());
}

TEST_F(MeshLoaderTestFixture, loading_solidworks_binary_stl) {
  // In general, binary STL files should not start with "solid" as this hints ASCII STL files.
  // Annoyingly, STL files exported from Solidworks don't follow this guideline and contain
  // "solid" at the start of binary STL files.
  // However, they don't finish with "endsolid" like ASCII STL files, so we can still detect
  // them as binary STL files.
  // This test checks that SOLIDWORKS binary STL files get loaded correctly and don't get treated
  // as ASCII STL files.
  std::string mesh_path = "package://rviz_rendering_tests/test_meshes/solidworks.stl";

  ASSERT_TRUE(rviz_rendering::loadMeshFromResource(mesh_path));
}
