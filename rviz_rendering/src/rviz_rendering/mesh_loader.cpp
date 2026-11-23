// Copyright (c) 2010, Willow Garage, Inc.
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


#include "rviz_rendering/mesh_loader.hpp"

#include <algorithm>
#include <cstdint>
#include <deque>
#include <exception>
#include <functional>
#include <list>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

#include "OgreHardwareBufferManager.h"
#include "OgreMaterial.h"
#include "OgreMaterialManager.h"
#include "OgreMeshManager.h"
#include "OgreMeshSerializer.h"
#include "OgrePass.h"
#include "OgreSubMesh.h"
#include "OgreTechnique.h"
#include "OgreTextureManager.h"
#include "OgreVector.h"

#include <QDir>  // NOLINT cpplint cannot handle include order here
#include <QFileInfo>  // NOLINT cpplint cannot handle include order here
#include <QString>  // NOLINT cpplint cannot handle include order here

#define ASSIMP_UNIFIED_HEADER_NAMES 1
#if defined(ASSIMP_UNIFIED_HEADER_NAMES)
#include "assimp/Importer.hpp"
#include "assimp/scene.h"
#include "assimp/postprocess.h"
#include "assimp/IOStream.hpp"
#include "assimp/IOSystem.hpp"
#else
#include "assimp/assimp.hpp"
#include "assimp/aiScene.h"
#include "assimp/aiPostProcess.h"
#include "assimp/IOStream.h"
#include "assimp/IOSystem.h"
#endif

#include "resource_retriever/plugins/retriever_plugin.hpp"
#include "resource_retriever/retriever.hpp"

#include "mesh_loader_helpers/assimp_loader.hpp"
#include "rviz_rendering/logging.hpp"

namespace rviz_rendering
{

namespace
{
// Meshes loaded from memory are reference counted and removed once unused. Unused meshes are
// kept up to a size limit, so that data which is sent only once can be loaded again.
constexpr size_t kMaxUnusedBytes = 64 * 1024 * 1024;
constexpr size_t kMaxInvalidPayloads = 16;

// Identifies mesh data without keeping a copy of it.
struct MeshDataDigest
{
  size_t hash;  // Part of the mesh name if the resource name is taken.
  uint64_t check;  // An independent hash, so that a matching hash alone is not enough.
  size_t size;

  bool operator==(const MeshDataDigest & other) const
  {
    return hash == other.hash && check == other.check && size == other.size;
  }
};

struct MemoryMesh
{
  std::string resource_name;  // The name the mesh was sent under.
  MeshDataDigest digest;
  size_t users;
  // Created for this mesh, and removed with it.
  std::vector<std::string> materials;
  std::vector<std::string> textures;
};

struct MemoryMeshes
{
  // Keyed by the name of the Ogre mesh. This is the resource name, unless that name is taken.
  std::unordered_map<std::string, MemoryMesh> by_name;
  // The mesh whose data was sent last for a resource name, for messages without the data.
  std::unordered_map<std::string, std::string> newest;
  // Meshes without users, oldest first.
  std::list<std::string> unused;
  size_t unused_bytes = 0;
  // Data that could not be loaded, so that it is not parsed and logged again.
  std::deque<MeshDataDigest> invalid_payloads;
};

MemoryMeshes & memoryMeshes()
{
  static MemoryMeshes meshes;
  return meshes;
}

bool meshExists(const std::string & name)
{
  return Ogre::MeshManager::getSingleton().resourceExists(name, ROS_PACKAGE_NAME);
}

MeshDataDigest digestMeshData(const std::string & filename, const std::vector<uint8_t> & data)
{
  const std::string_view bytes(reinterpret_cast<const char *>(data.data()), data.size());
  // 64 bit FNV-1a, over the file name and the data.
  uint64_t check = 14695981039346656037ull;
  for (const std::string_view part : {std::string_view(filename), bytes}) {
    for (const char c : part) {
      check = (check ^ static_cast<uint8_t>(c)) * 1099511628211ull;
    }
  }
  const size_t hash =
    std::hash<std::string_view>{}(bytes) ^ (std::hash<std::string>{}(filename) << 1);
  return MeshDataDigest{hash, check, data.size()};
}

/// Import a mesh in Ogre's own format. Throws if the data can't be read.
Ogre::MeshPtr importOgreMesh(const std::string & name, const uint8_t * data, size_t size)
{
  Ogre::MeshSerializer serializer;
  Ogre::DataStreamPtr stream(
    new Ogre::MemoryDataStream(const_cast<uint8_t *>(data), size));
  Ogre::MeshPtr mesh = Ogre::MeshManager::getSingleton().createManual(name, ROS_PACKAGE_NAME);
  serializer.importMesh(stream, mesh.get());
  stream->close();
  return mesh;
}

/// Remove a mesh created by loadMeshFromMemory, together with its materials and textures.
void removeMemoryMeshResources(
  const std::string & mesh_name, const std::vector<std::string> & materials,
  const std::vector<std::string> & textures)
{
  auto & material_manager = Ogre::MaterialManager::getSingleton();
  for (const auto & material : materials) {
    if (material_manager.resourceExists(material, ROS_PACKAGE_NAME)) {
      material_manager.remove(material, ROS_PACKAGE_NAME);
    }
  }
  auto & texture_manager = Ogre::TextureManager::getSingleton();
  for (const auto & texture : textures) {
    if (texture_manager.resourceExists(texture, ROS_PACKAGE_NAME)) {
      texture_manager.remove(texture, ROS_PACKAGE_NAME);
    }
  }
  if (meshExists(mesh_name)) {
    Ogre::MeshManager::getSingleton().remove(mesh_name, ROS_PACKAGE_NAME);
  }
}

/// Forget a mesh and remove it from Ogre. It must not be used anymore.
void removeMemoryMesh(const std::string & mesh_name)
{
  auto & meshes = memoryMeshes();
  auto mesh = meshes.by_name.find(mesh_name);
  if (mesh == meshes.by_name.end()) {
    return;
  }
  if (mesh->second.users == 0) {
    meshes.unused.remove(mesh_name);
    meshes.unused_bytes -= mesh->second.digest.size;
  }
  auto newest = meshes.newest.find(mesh->second.resource_name);
  if (newest != meshes.newest.end() && newest->second == mesh_name) {
    meshes.newest.erase(newest);
  }
  const MemoryMesh removed = std::move(mesh->second);
  meshes.by_name.erase(mesh);
  removeMemoryMeshResources(mesh_name, removed.materials, removed.textures);
}

/// Add a user to a mesh that is already loaded.
/**
 * \param data_was_sent Whether the caller sent the data, which makes it the newest mesh for its
 *   resource name. Loading a mesh without data must not change which mesh other callers get.
 */
Ogre::MeshPtr useMemoryMesh(const std::string & mesh_name, bool data_was_sent)
{
  auto & meshes = memoryMeshes();
  MemoryMesh & mesh = meshes.by_name.at(mesh_name);
  if (mesh.users++ == 0) {
    meshes.unused.remove(mesh_name);
    meshes.unused_bytes -= mesh.digest.size;
  }
  if (data_was_sent) {
    meshes.newest[mesh.resource_name] = mesh_name;
  }
  return Ogre::MeshManager::getSingleton().getByName(mesh_name, ROS_PACKAGE_NAME);
}

/// Whether the mesh is loaded from exactly this data.
bool holdsMemoryMesh(const std::string & mesh_name, const MeshDataDigest & digest)
{
  auto & meshes = memoryMeshes();
  auto mesh = meshes.by_name.find(mesh_name);
  if (mesh == meshes.by_name.end()) {
    return false;
  }
  if (!meshExists(mesh_name)) {
    // Ogre lost the mesh, e.g. when RViz is reset. Whoever used it still releases it under this
    // name, so the entry stays until then, and the name is not given to another mesh.
    if (mesh->second.users == 0) {
      removeMemoryMesh(mesh_name);
    }
    return false;
  }
  return mesh->second.digest == digest;
}

/// Whether a mesh that was sent under resource_name can be used.
bool isLoadedMemoryMesh(const std::string & mesh_name, const std::string & resource_name)
{
  auto & meshes = memoryMeshes();
  auto mesh = meshes.by_name.find(mesh_name);
  return mesh != meshes.by_name.end() && mesh->second.resource_name == resource_name &&
         meshExists(mesh_name);
}

/// Remove unused meshes until they fit the size limit, oldest first. The mesh that was released
/// last is always kept, since it is the one that is loaded again when a marker is updated.
void trimUnusedMeshes()
{
  auto & meshes = memoryMeshes();

  // Meshes that Ogre has lost are of no use and should not push out the ones that are still there.
  std::vector<std::string> lost;
  for (const auto & name : meshes.unused) {
    if (!meshExists(name)) {
      lost.push_back(name);
    }
  }
  for (const auto & name : lost) {
    removeMemoryMesh(name);
  }

  while (meshes.unused_bytes > kMaxUnusedBytes && meshes.unused.size() > 1) {
    const std::string oldest = meshes.unused.front();
    removeMemoryMesh(oldest);
  }
}

// Only data that fails because of what it contains is remembered. Failures that depend on the
// current state, such as a name that is taken, are tried again with the next message.
bool isKnownInvalid(const MeshDataDigest & digest)
{
  const auto & invalid = memoryMeshes().invalid_payloads;
  return std::find(invalid.begin(), invalid.end(), digest) != invalid.end();
}

void rememberInvalid(const MeshDataDigest & digest)
{
  auto & invalid = memoryMeshes().invalid_payloads;
  if (invalid.size() >= kMaxInvalidPayloads) {
    invalid.pop_front();
  }
  invalid.push_back(digest);
}
}  // namespace

Ogre::MeshPtr loadMeshFromResource(
  resource_retriever::Retriever * retriever,
  const std::string & resource_uri)
{
  // Early exit with empty retriever
  if (nullptr == retriever) {
    RVIZ_RENDERING_LOG_ERROR("retriever is unexpectedly nullptr");
    return nullptr;
  }

  // Meshes loaded from memory count their users, and handing them out here would not.
  if (memoryMeshes().by_name.count(resource_uri) > 0) {
    RVIZ_RENDERING_LOG_ERROR_STREAM(
      "Could not load resource [" << resource_uri << "]: it is a mesh loaded from memory, use "
        "loadMeshFromMemory()");
    return nullptr;
  }

  // Check for cached resources
  if (Ogre::MeshManager::getSingleton().resourceExists(resource_uri, ROS_PACKAGE_NAME)) {
    return Ogre::MeshManager::getSingleton().getByName(resource_uri, ROS_PACKAGE_NAME);
  }

  // Check for ".mesh" resources, which OGRE can load directly
  QFileInfo model_path(QString::fromStdString(resource_uri));
  std::string ext = model_path.completeSuffix().toStdString();
  if (ext == "mesh" || ext == "MESH") {
    auto res = retriever->get_shared(resource_uri);
    if (res == nullptr || res->data.empty()) {
      return nullptr;
    }
    return importOgreMesh(resource_uri, res->data.data(), res->data.size());
  }

  {
    AssimpLoader assimp_loader(retriever);
    const aiScene * scene = assimp_loader.getScene(resource_uri);
    if (scene == nullptr) {
      RVIZ_RENDERING_LOG_ERROR_STREAM(
        "Could not load resource [" << resource_uri.c_str() << "]: " <<
          assimp_loader.getErrorMessage());
      return nullptr;
    }
    return assimp_loader.meshFromAssimpScene(resource_uri, scene);
  }
}

Ogre::MeshPtr loadMeshFromMemory(
  const std::string & resource_name,
  const std::string & filename,
  const std::vector<uint8_t> & data,
  const std::string & previous_mesh_name,
  bool * data_is_invalid)
{
  auto & meshes = memoryMeshes();
  bool invalid = false;
  if (data_is_invalid == nullptr) {
    data_is_invalid = &invalid;
  }
  *data_is_invalid = false;

  if (data.empty()) {
    // The caller's own mesh comes first, so that it does not switch to data another caller sent
    // under the same name.
    if (!previous_mesh_name.empty() && isLoadedMemoryMesh(previous_mesh_name, resource_name)) {
      return useMemoryMesh(previous_mesh_name, false);
    }
    // Retried with every message and the data may come later, so this is not logged as an error.
    auto newest = meshes.newest.find(resource_name);
    if (newest != meshes.newest.end() && meshExists(newest->second)) {
      return useMemoryMesh(newest->second, false);
    }
    RVIZ_RENDERING_LOG_DEBUG_STREAM(
      "Could not load embedded mesh [" << resource_name << "]: no mesh data was provided");
    return nullptr;
  }

  const MeshDataDigest digest = digestMeshData(filename, data);
  const std::string hashed_name = resource_name + "#" + std::to_string(digest.hash);
  const std::string candidates[] = {resource_name, hashed_name};

  for (const auto & candidate : candidates) {
    if (holdsMemoryMesh(candidate, digest)) {
      return useMemoryMesh(candidate, true);
    }
  }

  if (isKnownInvalid(digest)) {
    RVIZ_RENDERING_LOG_DEBUG_STREAM(
      "Could not load embedded mesh [" << resource_name << "]: the data was invalid before");
    *data_is_invalid = true;
    return nullptr;
  }

  // The extension of the file name (not of the resource name) determines the file format, and
  // with it format specific conversions such as glTF's Y-up to Z-up.
  const std::string ext = lowercaseExtension(filename);
  if (ext.empty()) {
    RVIZ_RENDERING_LOG_ERROR_STREAM(
      "Could not load embedded mesh [" << resource_name << "]: file name [" << filename <<
        "] has no extension to determine the file format from");
    rememberInvalid(digest);
    *data_is_invalid = true;
    return nullptr;
  }

  // Ogre's own format, which Assimp can't read, is imported by Ogre directly.
  const bool is_ogre_mesh = ext == ".mesh";
  AssimpLoader assimp_loader(nullptr);
  const aiScene * scene = nullptr;
  if (!is_ogre_mesh) {
    scene = assimp_loader.getSceneFromMemory(filename, data.data(), data.size());
    if (scene == nullptr) {
      RVIZ_RENDERING_LOG_ERROR_STREAM(
        "Could not load embedded mesh [" << resource_name << "] (file name [" << filename <<
          "]): " << assimp_loader.getErrorMessage());
      rememberInvalid(digest);
      *data_is_invalid = true;
      return nullptr;
    }
  }

  // Take the first name that is free. A mesh without users is replaced, but meshes that are in
  // use, or that were not created here, are left alone.
  std::string mesh_name;
  for (const auto & candidate : candidates) {
    auto existing = meshes.by_name.find(candidate);
    if (existing != meshes.by_name.end() && existing->second.users == 0) {
      removeMemoryMesh(candidate);
    }
    if (meshes.by_name.count(candidate) == 0 && !meshExists(candidate)) {
      mesh_name = candidate;
      break;
    }
  }
  if (mesh_name.empty()) {
    RVIZ_RENDERING_LOG_ERROR_STREAM(
      "Could not load embedded mesh [" << resource_name << "]: the name is already in use");
    return nullptr;
  }

  Ogre::MeshPtr mesh;
  try {
    mesh = is_ogre_mesh ?
      importOgreMesh(mesh_name, data.data(), data.size()) :
      assimp_loader.meshFromAssimpScene(mesh_name, scene, ext);
  } catch (const std::exception & e) {
    RVIZ_RENDERING_LOG_ERROR_STREAM(
      "Could not load embedded mesh [" << resource_name << "]: " << e.what());
  }
  if (!mesh) {
    // E.g. a scene without meshes. Like data that can't be parsed, this is a problem of the data.
    removeMemoryMeshResources(
      mesh_name, assimp_loader.getMaterialNames(), assimp_loader.getTextureNames());
    rememberInvalid(digest);
    *data_is_invalid = true;
    return nullptr;
  }

  meshes.by_name[mesh_name] = MemoryMesh{
    resource_name, digest, 1, assimp_loader.getMaterialNames(), assimp_loader.getTextureNames()};
  meshes.newest[resource_name] = mesh_name;
  return mesh;
}

void releaseMeshFromMemory(const std::string & mesh_name)
{
  auto & meshes = memoryMeshes();
  auto mesh = meshes.by_name.find(mesh_name);
  if (mesh == meshes.by_name.end() || mesh->second.users == 0) {
    return;
  }
  if (--mesh->second.users > 0) {
    return;
  }

  meshes.unused.push_back(mesh_name);
  meshes.unused_bytes += mesh->second.digest.size;
  trimUnusedMeshes();
}

}  // namespace rviz_rendering
