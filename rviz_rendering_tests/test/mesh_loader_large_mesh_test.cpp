// Copyright (c) 2026, Open Source Robotics Foundation, Inc.
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

#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <OgreHardwareIndexBuffer.h>
#include <OgreSubMesh.h>
#include "resource_retriever/retriever.hpp"

#include "ogre_testing_environment.hpp"
#include "rviz_rendering/mesh_loader.hpp"

class LargeMeshLoaderTestFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    testing_environment_ = std::make_shared<rviz_rendering_tests::OgreTestingEnvironment>();
    testing_environment_->setUpOgreTestEnvironment();
  }

  std::shared_ptr<rviz_rendering_tests::OgreTestingEnvironment> testing_environment_;
  resource_retriever::Retriever retriever_;
};

namespace
{

template<typename T>
void append(std::vector<uint8_t> & out, T value)
{
  // glTF binary payloads are little endian; test hosts are little endian.
  const auto * bytes = reinterpret_cast<const uint8_t *>(&value);
  out.insert(out.end(), bytes, bytes + sizeof(T));
}

void padTo4(std::vector<uint8_t> & out, uint8_t pad)
{
  while (out.size() % 4 != 0) {
    out.push_back(pad);
  }
}

// Writes a binary glTF (.glb) grid of `side` x `side` vertices triangulated into
// 2 * (side - 1)^2 faces. The vertices carry explicit normals, so assimp keeps
// the geometry indexed (unlike STL, or a PLY with no normals, which get one
// vertex per face corner because aiProcess_GenNormals then splits them). With
// side = 250 the loaded mesh therefore has 62500 vertices, below the 16 bit
// index buffer threshold of 65536, but 124002 faces, above it: exactly the
// combination that selects the 16 bit index path in createAndFillIndexBuffer().
std::filesystem::path writeGridGlb(unsigned int side)
{
  const uint32_t vertex_count = side * side;
  const uint32_t face_count = 2 * (side - 1) * (side - 1);
  const uint32_t index_count = 3 * face_count;

  // Binary buffer: positions, then normals, then indices. Each element is
  // 4 bytes, so every view stays naturally 4-byte aligned.
  std::vector<uint8_t> bin;
  for (unsigned int y = 0; y < side; ++y) {
    for (unsigned int x = 0; x < side; ++x) {
      append<float>(bin, static_cast<float>(x));
      append<float>(bin, static_cast<float>(y));
      append<float>(bin, 0.0f);
    }
  }
  for (uint32_t i = 0; i < vertex_count; ++i) {
    append<float>(bin, 0.0f);
    append<float>(bin, 0.0f);
    append<float>(bin, 1.0f);
  }
  for (unsigned int y = 0; y + 1 < side; ++y) {
    for (unsigned int x = 0; x + 1 < side; ++x) {
      const uint32_t a = y * side + x;
      const uint32_t b = a + 1;
      const uint32_t c = a + side;
      const uint32_t d = c + 1;
      append<uint32_t>(bin, a);
      append<uint32_t>(bin, b);
      append<uint32_t>(bin, c);
      append<uint32_t>(bin, b);
      append<uint32_t>(bin, d);
      append<uint32_t>(bin, c);
    }
  }

  const uint32_t positions_len = vertex_count * 3 * sizeof(float);
  const uint32_t normals_len = vertex_count * 3 * sizeof(float);
  const uint32_t indices_len = index_count * sizeof(uint32_t);
  const uint32_t max_coord = side - 1;

  std::ostringstream json;
  json << "{"
       << "\"asset\":{\"version\":\"2.0\"},"
       << "\"scene\":0,"
       << "\"scenes\":[{\"nodes\":[0]}],"
       << "\"nodes\":[{\"mesh\":0}],"
       << "\"meshes\":[{\"primitives\":[{"
       << "\"attributes\":{\"POSITION\":0,\"NORMAL\":1},\"indices\":2,\"mode\":4}]}],"
       << "\"buffers\":[{\"byteLength\":" << bin.size() << "}],"
       << "\"bufferViews\":["
       << "{\"buffer\":0,\"byteOffset\":0,\"byteLength\":" << positions_len
       << ",\"target\":34962},"
       << "{\"buffer\":0,\"byteOffset\":" << positions_len << ",\"byteLength\":" << normals_len
       << ",\"target\":34962},"
       << "{\"buffer\":0,\"byteOffset\":" << (positions_len + normals_len)
       << ",\"byteLength\":" << indices_len << ",\"target\":34963}],"
       << "\"accessors\":["
       << "{\"bufferView\":0,\"componentType\":5126,\"count\":" << vertex_count
       << ",\"type\":\"VEC3\",\"min\":[0,0,0],\"max\":[" << max_coord << "," << max_coord << ",0]},"
       << "{\"bufferView\":1,\"componentType\":5126,\"count\":" << vertex_count
       << ",\"type\":\"VEC3\"},"
       << "{\"bufferView\":2,\"componentType\":5125,\"count\":" << index_count
       << ",\"type\":\"SCALAR\"}]"
       << "}";
  std::string json_str = json.str();

  std::vector<uint8_t> json_chunk(json_str.begin(), json_str.end());
  padTo4(json_chunk, ' ');
  padTo4(bin, 0);

  const uint32_t total_length =
    12 + 8 + static_cast<uint32_t>(json_chunk.size()) + 8 + static_cast<uint32_t>(bin.size());

  std::vector<uint8_t> glb;
  append<uint32_t>(glb, 0x46546C67);  // "glTF"
  append<uint32_t>(glb, 2);           // version
  append<uint32_t>(glb, total_length);
  append<uint32_t>(glb, static_cast<uint32_t>(json_chunk.size()));
  append<uint32_t>(glb, 0x4E4F534A);  // "JSON"
  glb.insert(glb.end(), json_chunk.begin(), json_chunk.end());
  append<uint32_t>(glb, static_cast<uint32_t>(bin.size()));
  append<uint32_t>(glb, 0x004E4942);  // "BIN\0"
  glb.insert(glb.end(), bin.begin(), bin.end());

  auto path = std::filesystem::temp_directory_path() / "rviz_rendering_large_grid.glb";
  std::ofstream out(path, std::ios::binary);
  out.write(reinterpret_cast<const char *>(glb.data()), glb.size());
  return path;
}

}  // namespace

TEST_F(
  LargeMeshLoaderTestFixture,
  assimp_loader_handles_more_than_65535_faces_with_16_bit_indices)
{
  // Regression test: fillIndexBuffer<uint16_t>() used the index element type
  // as its loop counter, so a mesh with fewer than 65536 vertices (16 bit
  // index buffer) but 65536 or more faces made the counter wrap around and
  // write past the end of the index buffer instead of terminating.
  const unsigned int side = 250;
  const size_t expected_vertices = side * side;               // 62500
  const size_t expected_faces = 2 * (side - 1) * (side - 1);  // 124002

  auto glb_path = writeGridGlb(side);
  std::string mesh_path = "file://" + glb_path.string();

  auto mesh = rviz_rendering::loadMeshFromResource(&this->retriever_, mesh_path);
  std::filesystem::remove(glb_path);

  ASSERT_TRUE(mesh);
  ASSERT_EQ(1u, mesh->getNumSubMeshes());
  auto * submesh = mesh->getSubMesh(0);
  ASSERT_EQ(expected_vertices, submesh->vertexData->vertexCount);
  ASSERT_EQ(
    Ogre::HardwareIndexBuffer::IT_16BIT, submesh->indexData->indexBuffer->getType());
  ASSERT_EQ(expected_faces * 3, submesh->indexData->indexCount);
}
