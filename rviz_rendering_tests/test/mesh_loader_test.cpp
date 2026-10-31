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
#include <OgreTechnique.h>
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

  // Empty if the file can't be read, so tests can ASSERT on it.
  std::vector<uint8_t> readFile(const std::string & uri)
  {
    auto res = retriever_.get_shared(uri);
    if (res == nullptr) {
      return {};
    }
    return std::vector<uint8_t>(res->data.begin(), res->data.end());
  }

  std::shared_ptr<rviz_rendering_tests::OgreTestingEnvironment> testing_environment_;
  resource_retriever::Retriever retriever_;
};

namespace
{
// A triangle (0, 0, 0), (1, 0, 0), (0, 2, 0) in glTF's Y-up convention, with its buffer inlined as
// a data URI. In RViz's Z-up convention it spans 2 units along z and none along y.
const char kTriangleGltf[] =
  "{\"asset\":{\"version\":\"2.0\"},\"scene\":0,\"scenes\":[{\"nodes\":[0]}],"
  "\"nodes\":[{\"mesh\":0}],\"meshes\":[{\"primitives\":[{\"attributes\":{\"POSITION\":0}}]}],"
  "\"accessors\":[{\"bufferView\":0,\"componentType\":5126,\"count\":3,\"type\":\"VEC3\","
  "\"min\":[0,0,0],\"max\":[1,2,0]}],\"bufferViews\":[{\"buffer\":0,\"byteLength\":36}],"
  "\"buffers\":[{\"byteLength\":36,\"uri\":\"data:application/octet-stream;base64,"
  "AAAAAAAAAAAAAAAAAACAPwAAAAAAAAAAAAAAAAAAAEAAAAAA\"}]}";

// The same triangle, with a texture that is not part of the data.
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

// Ogre pads mesh bounds a bit, so compare with a tolerance.
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
  ASSERT_FALSE(data.empty());
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
  ASSERT_FALSE(data.empty());
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
  ASSERT_FALSE(data.empty());

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
  // The resource name has no extension, but the glTF Y-up to Z-up conversion must still apply.
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
  ASSERT_FALSE(data.empty());
  std::string name = "embedded://memory_cached_f2";

  auto first = rviz_rendering::loadMeshFromMemory(name, "F2.stl", data);
  // Once loaded, the data is not needed again.
  auto second = rviz_rendering::loadMeshFromMemory(name, "F2.stl", {});

  ASSERT_TRUE(first);
  ASSERT_TRUE(second);
  ASSERT_EQ(first, second);
}

TEST_F(MeshLoaderTestFixture, loading_mesh_from_memory_with_external_texture_does_not_crash) {
  // Files the mesh refers to can't be resolved from memory, but the mesh should still load.
  std::vector<uint8_t> data(
    kTriangleWithExternalTextureGltf,
    kTriangleWithExternalTextureGltf + sizeof(kTriangleWithExternalTextureGltf) - 1);

  auto mesh = rviz_rendering::loadMeshFromMemory(
    "embedded://memory_triangle_external_texture", "triangle.gltf", data);

  ASSERT_TRUE(mesh);
  assertTriangleIsZUp(mesh);
  auto material = Ogre::MaterialManager::getSingleton().getByName(
    mesh->getSubMesh(0)->getMaterialName(), "rviz_rendering");
  ASSERT_TRUE(material);
  // The missing texture is skipped, not left as a blank layer.
  EXPECT_EQ(0u, material->getTechnique(0)->getPass(0)->getNumTextureUnitStates());
}

TEST_F(MeshLoaderTestFixture, loading_mesh_from_memory_without_file_extension_fails) {
  // Assimp could parse the data, but format specific handling such as glTF's conversion to Z-up
  // would silently be skipped.
  std::vector<uint8_t> data(kTriangleGltf, kTriangleGltf + sizeof(kTriangleGltf) - 1);

  EXPECT_FALSE(rviz_rendering::loadMeshFromMemory("embedded://memory_no_filename", "", data));
  EXPECT_FALSE(
    rviz_rendering::loadMeshFromMemory("embedded://memory_no_extension", "triangle", data));
}

TEST_F(MeshLoaderTestFixture, loading_mesh_from_memory_with_new_data_replaces_an_unused_mesh) {
  std::vector<uint8_t> triangle(kTriangleGltf, kTriangleGltf + sizeof(kTriangleGltf) - 1);
  auto stl = readFile("package://rviz_rendering_tests/test_meshes/F2.stl");
  ASSERT_FALSE(stl.empty());
  std::string name = "embedded://memory_replaced";

  auto first = rviz_rendering::loadMeshFromMemory(name, "triangle.gltf", triangle);
  ASSERT_TRUE(first);
  rviz_rendering::releaseMeshFromMemory(first->getName());
  auto second = rviz_rendering::loadMeshFromMemory(name, "F2.stl", stl);

  ASSERT_TRUE(second);
  EXPECT_NE(first, second);
  EXPECT_EQ(name, second->getName());
  EXPECT_FLOAT_EQ(34.920441f, second->getBoundingSphereRadius());
  EXPECT_EQ(second, Ogre::MeshManager::getSingleton().getByName(name, "rviz_rendering"));
  rviz_rendering::releaseMeshFromMemory(second->getName());
}

TEST_F(MeshLoaderTestFixture, new_data_does_not_replace_a_mesh_from_memory_that_is_in_use) {
  std::vector<uint8_t> triangle(kTriangleGltf, kTriangleGltf + sizeof(kTriangleGltf) - 1);
  auto stl = readFile("package://rviz_rendering_tests/test_meshes/F2.stl");
  ASSERT_FALSE(stl.empty());
  std::string name = "embedded://memory_in_use";

  auto first = rviz_rendering::loadMeshFromMemory(name, "triangle.gltf", triangle);
  auto second = rviz_rendering::loadMeshFromMemory(name, "F2.stl", stl);

  ASSERT_TRUE(first);
  ASSERT_TRUE(second);
  EXPECT_NE(first->getName(), second->getName());
  // The first mesh and its materials are left alone.
  EXPECT_EQ(name, first->getName());
  EXPECT_EQ(first, Ogre::MeshManager::getSingleton().getByName(name, "rviz_rendering"));
  EXPECT_TRUE(
    Ogre::MaterialManager::getSingleton().resourceExists(name + "Material0", "rviz_rendering"));

  // Each mesh is released on its own.
  rviz_rendering::releaseMeshFromMemory(first->getName());
  EXPECT_TRUE(
    Ogre::MeshManager::getSingleton().resourceExists(second->getName(), "rviz_rendering"));
  rviz_rendering::releaseMeshFromMemory(second->getName());
}

TEST_F(MeshLoaderTestFixture, failing_to_load_new_data_does_not_affect_a_mesh_in_use) {
  std::vector<uint8_t> triangle(kTriangleGltf, kTriangleGltf + sizeof(kTriangleGltf) - 1);
  std::vector<uint8_t> garbage = {'n', 'o', 'p', 'e'};
  auto stl = readFile("package://rviz_rendering_tests/test_meshes/F2.stl");
  ASSERT_FALSE(stl.empty());
  std::string name = "embedded://memory_failing";

  auto first = rviz_rendering::loadMeshFromMemory(name, "triangle.gltf", triangle);
  ASSERT_TRUE(first);
  ASSERT_TRUE(rviz_rendering::loadMeshFromMemory(name, "triangle.gltf", {}));  // Second user.
  EXPECT_FALSE(rviz_rendering::loadMeshFromMemory(name, "other.gltf", garbage));
  EXPECT_EQ(first, Ogre::MeshManager::getSingleton().getByName(name, "rviz_rendering"));

  // The failure did not lose a user: one is left after a release, so new data gets its own mesh.
  rviz_rendering::releaseMeshFromMemory(name);
  auto other = rviz_rendering::loadMeshFromMemory(name, "F2.stl", stl);
  ASSERT_TRUE(other);
  EXPECT_NE(name, other->getName());
  rviz_rendering::releaseMeshFromMemory(name);
  rviz_rendering::releaseMeshFromMemory(other->getName());
}

TEST_F(MeshLoaderTestFixture, mesh_from_memory_does_not_replace_a_foreign_mesh) {
  std::vector<uint8_t> triangle(kTriangleGltf, kTriangleGltf + sizeof(kTriangleGltf) - 1);
  std::string name = "embedded://memory_foreign";
  auto foreign = Ogre::MeshManager::getSingleton().createManual(name, "rviz_rendering");

  auto mesh = rviz_rendering::loadMeshFromMemory(name, "triangle.gltf", triangle);

  ASSERT_TRUE(mesh);
  EXPECT_NE(name, mesh->getName());
  EXPECT_EQ(foreign, Ogre::MeshManager::getSingleton().getByName(name, "rviz_rendering"));
  rviz_rendering::releaseMeshFromMemory(mesh->getName());
  Ogre::MeshManager::getSingleton().remove(foreign);
}

TEST_F(MeshLoaderTestFixture, released_mesh_from_memory_can_be_loaded_again_without_data) {
  auto data = readFile("package://rviz_rendering_tests/test_meshes/colored_box.glb");
  ASSERT_FALSE(data.empty());
  std::string name = "embedded://memory_released";

  auto first = rviz_rendering::loadMeshFromMemory(name, "colored_box.glb", data);
  ASSERT_TRUE(first);
  rviz_rendering::releaseMeshFromMemory(first->getName());
  auto again = rviz_rendering::loadMeshFromMemory(name, "colored_box.glb", {});

  EXPECT_EQ(first, again);
  rviz_rendering::releaseMeshFromMemory(again->getName());
}

TEST_F(MeshLoaderTestFixture, unused_meshes_from_memory_are_removed_beyond_a_size_limit) {
  // Large payloads that are cheap to parse: a glTF followed by white space.
  auto padded = [](size_t megabytes) {
      std::vector<uint8_t> data(kTriangleGltf, kTriangleGltf + sizeof(kTriangleGltf) - 1);
      data.resize(megabytes * 1024 * 1024, ' ');
      return data;
    };
  std::string old_name = "embedded://memory_old";
  std::string new_name = "embedded://memory_new";

  auto old_mesh = rviz_rendering::loadMeshFromMemory(old_name, "old.gltf", padded(40));
  ASSERT_TRUE(old_mesh);
  rviz_rendering::releaseMeshFromMemory(old_name);
  EXPECT_TRUE(Ogre::MeshManager::getSingleton().resourceExists(old_name, "rviz_rendering"));
  EXPECT_TRUE(
    Ogre::MaterialManager::getSingleton().resourceExists(old_name + "Material0", "rviz_rendering"));

  auto new_mesh = rviz_rendering::loadMeshFromMemory(new_name, "new.gltf", padded(41));
  ASSERT_TRUE(new_mesh);
  rviz_rendering::releaseMeshFromMemory(new_name);

  // Together they are over the limit, so the one that was released first is removed.
  EXPECT_FALSE(Ogre::MeshManager::getSingleton().resourceExists(old_name, "rviz_rendering"));
  EXPECT_FALSE(
    Ogre::MaterialManager::getSingleton().resourceExists(old_name + "Material0", "rviz_rendering"));
  EXPECT_FALSE(rviz_rendering::loadMeshFromMemory(old_name, "old.gltf", {}));
  EXPECT_TRUE(Ogre::MeshManager::getSingleton().resourceExists(new_name, "rviz_rendering"));
}
