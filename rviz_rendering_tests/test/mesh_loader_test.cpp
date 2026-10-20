// Copyright (c) 2018, Bosch Software Innovations GmbH.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <OgreRoot.h>
#include <OgreSubMesh.h>
#include <OgreMaterialManager.h>
#include <OgreMeshManager.h>
#include <OgreHardwareVertexBuffer.h>
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

  std::vector<uint8_t> readFile(const std::string & uri)
  {
    auto res = retriever_.get_shared(uri);
    return std::vector<uint8_t>(res->data.begin(), res->data.end());
  }

  std::shared_ptr<rviz_rendering_tests::OgreTestingEnvironment> testing_environment_;
  resource_retriever::Retriever retriever_;
};

namespace
{
// A single triangle with vertices (0, 0, 0), (1, 0, 0) and (0, 2, 0) in glTF's Y-up convention,
// stored as a self-contained .gltf file (the vertex buffer is embedded as a data URI).
// Once converted to RViz's Z-up convention, the triangle spans 2 units along the z-axis and none
// along the y-axis.
const char kTriangleGltf[] =
  "{\"asset\":{\"version\":\"2.0\"},\"scene\":0,\"scenes\":[{\"nodes\":[0]}],"
  "\"nodes\":[{\"mesh\":0}],\"meshes\":[{\"primitives\":[{\"attributes\":{\"POSITION\":0}}]}],"
  "\"accessors\":[{\"bufferView\":0,\"componentType\":5126,\"count\":3,\"type\":\"VEC3\","
  "\"min\":[0,0,0],\"max\":[1,2,0]}],\"bufferViews\":[{\"buffer\":0,\"byteLength\":36}],"
  "\"buffers\":[{\"byteLength\":36,\"uri\":\"data:application/octet-stream;base64,"
  "AAAAAAAAAAAAAAAAAACAPwAAAAAAAAAAAAAAAAAAAEAAAAAA\"}]}";

// Like kTriangleGltf, but its material refers to a texture file that is not part of the data.
const char kTriangleWithExternalTextureGltf[] =
  "{\"asset\":{\"version\":\"2.0\"},\"scene\":0,\"scenes\":[{\"nodes\":[0]}],"
  "\"nodes\":[{\"mesh\":0}],\"meshes\":[{\"primitives\":[{\"attributes\":{\"POSITION\":0},"
  "\"material\":0}]}],"
  "\"materials\":[{\"pbrMetallicRoughness\":{\"baseColorTexture\":{\"index\":0}}}],"
  "\"textures\":[{\"source\":0}],\"images\":[{\"uri\":\"missing_texture.png\"}],"
  "\"accessors\":[{\"bufferView\":0,\"componentType\":5126,\"count\":3,\"type\":\"VEC3\","
  "\"min\":[0,0,0],\"max\":[1,2,0]}],\"bufferViews\":[{\"buffer\":0,\"byteLength\":36}],"
  "\"buffers\":[{\"byteLength\":36,\"uri\":\"data:application/octet-stream;base64,"
  "AAAAAAAAAAAAAAAAAACAPwAAAAAAAAAAAAAAAAAAAEAAAAAA\"}]}";

// Ogre pads the bounds of a mesh slightly, therefore compare with a tolerance.
void assertTriangleIsZUp(const Ogre::MeshPtr & mesh)
{
  const Ogre::Vector3 size = mesh->getBounds().getSize();
  EXPECT_NEAR(size.x, 1.0f, 0.1f);
  EXPECT_NEAR(size.y, 0.0f, 0.1f);
  EXPECT_NEAR(size.z, 2.0f, 0.1f);
}
}  // namespace

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

  EXPECT_THROW(
    rviz_rendering::loadMeshFromResource(&this->retriever_, mesh_path),
    resource_retriever::Exception);
}

TEST_F(MeshLoaderTestFixture, can_load_ogre_mesh_files) {
  std::string mesh_path = "package://rviz_rendering/ogre_media/models/rviz_sphere.mesh";

  auto mesh = rviz_rendering::loadMeshFromResource(&this->retriever_, mesh_path);

  ASSERT_TRUE(mesh->isManuallyLoaded());
  ASSERT_EQ(mesh_path, mesh->getName());
}

TEST_F(MeshLoaderTestFixture, can_load_stl_files) {
  std::string mesh_path = "package://rviz_rendering_tests/test_meshes/F2.stl";

  auto mesh = rviz_rendering::loadMeshFromResource(&this->retriever_, mesh_path);

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

  ASSERT_FALSE(rviz_rendering::loadMeshFromResource(&this->retriever_, mesh_path));
}

TEST_F(MeshLoaderTestFixture, loading_invalid_stl_files_fail) {
  /// Load an invalid STL binary file (size does not match the expected size).
  std::string mesh_path = "package://rviz_rendering_tests/test_meshes/invalid.stl";

  ASSERT_FALSE(rviz_rendering::loadMeshFromResource(&this->retriever_, mesh_path));
}

TEST_F(MeshLoaderTestFixture, loading_invalid_ascii_stl_file) {
  /// Load an invalid STL binary file (size does not match the expected size).
  std::string mesh_path = "package://rviz_rendering_tests/test_meshes/invalid_ascii.stl";

  ASSERT_FALSE(rviz_rendering::loadMeshFromResource(&this->retriever_, mesh_path));
}

TEST_F(MeshLoaderTestFixture, loading_invalid_stl_files_should_fail) {
  /// Load an invalid STL binary file (size does not match the expected size,
  /// but does if incorrectly read as an 16-bit uint)
  std::string mesh_path =
    "package://rviz_rendering_tests/test_meshes/16bit_vs_32bit_should_fail.stl";

  ASSERT_FALSE(rviz_rendering::loadMeshFromResource(&this->retriever_, mesh_path));
}

TEST_F(MeshLoaderTestFixture, loading_almost_invalid_stl_files_should_fail) {
  /// Load a "potentially" valid STL binary file with bigger size than the
  /// expected. The file will not load.
  std::string mesh_path = "package://rviz_rendering_tests/test_meshes/invalid_extra.stl";

  EXPECT_FALSE(rviz_rendering::loadMeshFromResource(&this->retriever_, mesh_path));
}

TEST_F(MeshLoaderTestFixture, loading_stl_mesh_twice_should_not_fail) {
  std::string mesh_path = "package://rviz_rendering_tests/test_meshes/F2.stl";

  ASSERT_TRUE(rviz_rendering::loadMeshFromResource(&this->retriever_, mesh_path));
  ASSERT_TRUE(rviz_rendering::loadMeshFromResource(&this->retriever_, mesh_path));
}

TEST_F(MeshLoaderTestFixture, can_load_assimp_mesh_files) {
  std::string mesh_path = "package://rviz_rendering_tests/test_meshes/pr2-base.dae";

  auto mesh = rviz_rendering::loadMeshFromResource(&this->retriever_, mesh_path);

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

  auto mesh = rviz_rendering::loadMeshFromResource(&this->retriever_, mesh_path);

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

  ASSERT_TRUE(rviz_rendering::loadMeshFromResource(&this->retriever_, mesh_path));
}

TEST_F(MeshLoaderTestFixture, assimp_loader_reads_per_vertex_colors) {
  // A glb cube whose vertices carry per-vertex colours. The loader must add a
  // diffuse (colour) vertex element so the colours are rendered.
  std::string mesh_path = "package://rviz_rendering_tests/test_meshes/colored_box.glb";

  auto mesh = rviz_rendering::loadMeshFromResource(&this->retriever_, mesh_path);

  ASSERT_TRUE(mesh);
  ASSERT_GT(mesh->getNumSubMeshes(), 0u);
  const auto * declaration = mesh->getSubMesh(0)->vertexData->vertexDeclaration;
  ASSERT_NE(nullptr, declaration->findElementBySemantic(Ogre::VES_DIFFUSE));
}

TEST_F(MeshLoaderTestFixture, meshes_without_vertex_colors_have_no_diffuse_element) {
  // A plain STL has no per-vertex colours, so no diffuse vertex element is added.
  std::string mesh_path = "package://rviz_rendering_tests/test_meshes/F2.stl";

  auto mesh = rviz_rendering::loadMeshFromResource(&this->retriever_, mesh_path);

  ASSERT_TRUE(mesh);
  const auto * declaration = mesh->getSubMesh(0)->vertexData->vertexDeclaration;
  ASSERT_EQ(nullptr, declaration->findElementBySemantic(Ogre::VES_DIFFUSE));
}

TEST_F(MeshLoaderTestFixture, can_load_glb_mesh_from_memory) {
  auto data = readFile("package://rviz_rendering_tests/test_meshes/colored_box.glb");
  std::string name = "embedded://memory_colored_box";

  auto mesh = rviz_rendering::loadMeshFromMemory(name, "colored_box.glb", data);

  ASSERT_TRUE(mesh);
  ASSERT_TRUE(mesh->isManuallyLoaded());
  ASSERT_EQ(name, mesh->getName());
  ASSERT_GT(mesh->getNumSubMeshes(), 0u);
  const auto * declaration = mesh->getSubMesh(0)->vertexData->vertexDeclaration;
  ASSERT_NE(nullptr, declaration->findElementBySemantic(Ogre::VES_DIFFUSE));
}

TEST_F(MeshLoaderTestFixture, can_load_stl_mesh_from_memory) {
  auto data = readFile("package://rviz_rendering_tests/test_meshes/F2.stl");
  std::string name = "embedded://memory_f2";

  auto mesh = rviz_rendering::loadMeshFromMemory(name, "F2.stl", data);

  ASSERT_TRUE(mesh);
  size_t actual_vertex_count = 0;
  for (size_t i = 0; i < mesh->getNumSubMeshes(); ++i) {
    actual_vertex_count += mesh->getSubMesh(i)->vertexData->vertexCount;
  }
  ASSERT_EQ(name, mesh->getName());
  ASSERT_EQ(35532u, actual_vertex_count);
  ASSERT_FLOAT_EQ(34.920441f, mesh->getBoundingSphereRadius());
}

TEST_F(MeshLoaderTestFixture, can_load_assimp_mesh_from_memory_matches_file_loading) {
  auto data = readFile("package://rviz_rendering_tests/test_meshes/pr2-base.dae");

  auto mesh = rviz_rendering::loadMeshFromMemory(
    "embedded://memory_pr2_base", "pr2-base.dae", data);

  ASSERT_TRUE(mesh);
  // Same expectations as can_load_assimp_mesh_files
  ASSERT_EQ(3600u, mesh->getSubMesh(0)->vertexData->vertexCount);
  ASSERT_FLOAT_EQ(0.754754f, mesh->getBoundingSphereRadius());
  assertBoundingBoxEquality(
    Ogre::AxisAlignedBox(
      Ogre::Vector3(-0.342375f, -0.340043f, -0.00656401f),
      Ogre::Vector3(0.340425f, 0.340043f, 0.662748f)),
    mesh->getBounds());
}

TEST_F(MeshLoaderTestFixture, file_format_of_mesh_from_memory_is_taken_from_the_file_name) {
  // The name of the resource ("embedded://...") has no file extension. The glTF specific
  // conversion from Y-up to Z-up must still be applied based on the extension of the file name.
  std::vector<uint8_t> data(kTriangleGltf, kTriangleGltf + sizeof(kTriangleGltf) - 1);

  auto mesh = rviz_rendering::loadMeshFromMemory(
    "embedded://memory_triangle", "triangle.gltf", data);

  ASSERT_TRUE(mesh);
  assertTriangleIsZUp(mesh);
}

TEST_F(MeshLoaderTestFixture, file_extension_of_mesh_from_memory_is_case_insensitive) {
  std::vector<uint8_t> data(kTriangleGltf, kTriangleGltf + sizeof(kTriangleGltf) - 1);

  auto mesh = rviz_rendering::loadMeshFromMemory(
    "embedded://memory_triangle_upper", "TRIANGLE.GLTF", data);

  ASSERT_TRUE(mesh);
  assertTriangleIsZUp(mesh);
}

TEST_F(MeshLoaderTestFixture, loading_mesh_from_memory_without_data_fails) {
  EXPECT_FALSE(
    rviz_rendering::loadMeshFromMemory("embedded://memory_empty", "empty.stl", {}));
}

TEST_F(MeshLoaderTestFixture, loading_invalid_mesh_from_memory_fails) {
  std::vector<uint8_t> garbage = {'n', 'o', 't', ' ', 'a', ' ', 'm', 'e', 's', 'h'};

  EXPECT_FALSE(
    rviz_rendering::loadMeshFromMemory("embedded://memory_garbage_stl", "mesh.stl", garbage));
  EXPECT_FALSE(
    rviz_rendering::loadMeshFromMemory("embedded://memory_garbage_dae", "mesh.dae", garbage));
  EXPECT_FALSE(
    rviz_rendering::loadMeshFromMemory("embedded://memory_garbage_none", "", garbage));
}

TEST_F(MeshLoaderTestFixture, loading_mesh_from_memory_twice_returns_the_cached_mesh) {
  auto data = readFile("package://rviz_rendering_tests/test_meshes/F2.stl");
  std::string name = "embedded://memory_cached_f2";

  auto first = rviz_rendering::loadMeshFromMemory(name, "F2.stl", data);
  // The data is not needed anymore once the mesh has been loaded under this name.
  auto second = rviz_rendering::loadMeshFromMemory(name, "F2.stl", {});

  ASSERT_TRUE(first);
  ASSERT_TRUE(second);
  ASSERT_EQ(first, second);
}

TEST_F(MeshLoaderTestFixture, loading_mesh_from_memory_with_external_texture_does_not_crash) {
  // Files that are referenced by a mesh from memory cannot be resolved. The mesh itself should
  // still load.
  std::vector<uint8_t> data(
    kTriangleWithExternalTextureGltf,
    kTriangleWithExternalTextureGltf + sizeof(kTriangleWithExternalTextureGltf) - 1);

  auto mesh = rviz_rendering::loadMeshFromMemory(
    "embedded://memory_triangle_external_texture", "triangle.gltf", data);

  ASSERT_TRUE(mesh);
  assertTriangleIsZUp(mesh);
}

TEST_F(MeshLoaderTestFixture, loading_mesh_from_memory_without_file_extension_fails) {
  // Assimp could parse the data, but format specific handling such as glTF's conversion to Z-up
  // would silently be skipped.
  std::vector<uint8_t> data(kTriangleGltf, kTriangleGltf + sizeof(kTriangleGltf) - 1);

  EXPECT_FALSE(rviz_rendering::loadMeshFromMemory("embedded://memory_no_filename", "", data));
  EXPECT_FALSE(
    rviz_rendering::loadMeshFromMemory("embedded://memory_no_extension", "triangle", data));
}

TEST_F(MeshLoaderTestFixture, loading_mesh_from_memory_with_new_data_replaces_the_mesh) {
  std::vector<uint8_t> triangle(kTriangleGltf, kTriangleGltf + sizeof(kTriangleGltf) - 1);
  auto stl = readFile("package://rviz_rendering_tests/test_meshes/F2.stl");
  std::string name = "embedded://memory_replaced";

  auto first = rviz_rendering::loadMeshFromMemory(name, "triangle.gltf", triangle);
  auto second = rviz_rendering::loadMeshFromMemory(name, "F2.stl", stl);

  ASSERT_TRUE(first);
  ASSERT_TRUE(second);
  EXPECT_NE(first, second);
  EXPECT_FLOAT_EQ(34.920441f, second->getBoundingSphereRadius());
  EXPECT_EQ(second, Ogre::MeshManager::getSingleton().getByName(name, "rviz_rendering"));
  rviz_rendering::releaseMeshFromMemory(name);
  rviz_rendering::releaseMeshFromMemory(name);
}

TEST_F(MeshLoaderTestFixture, released_mesh_from_memory_is_removed_with_its_materials) {
  auto data = readFile("package://rviz_rendering_tests/test_meshes/colored_box.glb");
  std::string name = "embedded://memory_released";

  ASSERT_TRUE(rviz_rendering::loadMeshFromMemory(name, "colored_box.glb", data));
  ASSERT_TRUE(rviz_rendering::loadMeshFromMemory(name, "colored_box.glb", {}));
  ASSERT_TRUE(
    Ogre::MaterialManager::getSingleton().resourceExists(name + "Material0", "rviz_rendering"));

  // The mesh is kept while it is still used.
  rviz_rendering::releaseMeshFromMemory(name);
  EXPECT_TRUE(Ogre::MeshManager::getSingleton().resourceExists(name, "rviz_rendering"));

  rviz_rendering::releaseMeshFromMemory(name);
  EXPECT_FALSE(Ogre::MeshManager::getSingleton().resourceExists(name, "rviz_rendering"));
  EXPECT_FALSE(
    Ogre::MaterialManager::getSingleton().resourceExists(name + "Material0", "rviz_rendering"));
  EXPECT_FALSE(rviz_rendering::loadMeshFromMemory(name, "colored_box.glb", {}));
}
