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
#include <cctype>
#include <exception>
#include <filesystem>
#include <functional>
#include <list>
#include <string>
#include <string_view>
#include <unordered_map>
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

Ogre::MeshPtr loadMeshFromResource(
  resource_retriever::Retriever * retriever,
  const std::string & resource_uri)
{
  // Early exit with empty retriever
  if (nullptr == retriever) {
    RVIZ_RENDERING_LOG_ERROR("retriever is unexpectedly nullptr");
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
    Ogre::MeshSerializer ser;
    Ogre::DataStreamPtr stream(new Ogre::MemoryDataStream(
      const_cast<void *>(reinterpret_cast<void const *>(res->data.data())), res->data.size()));
    Ogre::MeshPtr mesh = Ogre::MeshManager::getSingleton().createManual(resource_uri,
        ROS_PACKAGE_NAME);
    ser.importMesh(stream, mesh.get());
    stream->close();
    return mesh;
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

namespace
{
// Meshes loaded from memory are reference counted and removed once unused. Unused meshes are
// kept up to a size limit, so that data which is sent only once survives a reset.
constexpr size_t kMaxUnusedBytes = 64 * 1024 * 1024;

struct MemoryMesh
{
  std::string resource_name;  // The name the mesh was sent under.
  size_t data_hash;
  size_t data_size;
  size_t users;
};

struct MemoryMeshes
{
  // Keyed by the name of the Ogre mesh. This is the resource name, unless that name is taken.
  std::unordered_map<std::string, MemoryMesh> by_name;
  // The mesh loaded last for a resource name, for messages that do not send the data again.
  std::unordered_map<std::string, std::string> newest;
  // Meshes without users, oldest first.
  std::list<std::string> unused;
  size_t unused_bytes = 0;
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

size_t hashMeshData(const std::string & filename, const std::vector<uint8_t> & data)
{
  const size_t data_hash = std::hash<std::string_view>{}(
    std::string_view(reinterpret_cast<const char *>(data.data()), data.size()));
  return data_hash ^ (std::hash<std::string>{}(filename) << 1);
}

/// Remove a mesh created by loadMeshFromMemory, together with its materials and textures.
void removeMemoryMeshResources(const std::string & mesh_name)
{
  auto & material_manager = Ogre::MaterialManager::getSingleton();
  auto & texture_manager = Ogre::TextureManager::getSingleton();
  // The assimp loader names the materials of a mesh "<mesh name>Material<index>" and its
  // embedded textures "<mesh name><texture name>".
  for (size_t i = 0; ; ++i) {
    const std::string material_name = mesh_name + "Material" + std::to_string(i);
    Ogre::MaterialPtr material = material_manager.getByName(material_name, ROS_PACKAGE_NAME);
    if (!material) {
      break;
    }
    for (auto * technique : material->getTechniques()) {
      for (auto * pass : technique->getPasses()) {
        for (auto * texture_unit : pass->getTextureUnitStates()) {
          const std::string & texture_name = texture_unit->getTextureName();
          if (texture_name.rfind(mesh_name, 0) == 0 &&
            texture_manager.resourceExists(texture_name, ROS_PACKAGE_NAME))
          {
            texture_manager.remove(texture_name, ROS_PACKAGE_NAME);
          }
        }
      }
    }
    material_manager.remove(material);
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
  if (mesh != meshes.by_name.end()) {
    if (mesh->second.users == 0) {
      meshes.unused.remove(mesh_name);
      meshes.unused_bytes -= mesh->second.data_size;
    }
    auto newest = meshes.newest.find(mesh->second.resource_name);
    if (newest != meshes.newest.end() && newest->second == mesh_name) {
      meshes.newest.erase(newest);
    }
    meshes.by_name.erase(mesh);
  }
  removeMemoryMeshResources(mesh_name);
}

/// Add a user to a mesh that is already loaded.
Ogre::MeshPtr useMemoryMesh(const std::string & mesh_name)
{
  auto & meshes = memoryMeshes();
  MemoryMesh & mesh = meshes.by_name.at(mesh_name);
  if (mesh.users++ == 0) {
    meshes.unused.remove(mesh_name);
    meshes.unused_bytes -= mesh.data_size;
  }
  meshes.newest[mesh.resource_name] = mesh_name;
  return Ogre::MeshManager::getSingleton().getByName(mesh_name, ROS_PACKAGE_NAME);
}

/// Whether the mesh is loaded from exactly this data. Forgets meshes that Ogre has lost.
bool holdsMemoryMesh(const std::string & mesh_name, size_t data_hash, size_t data_size)
{
  auto & meshes = memoryMeshes();
  auto mesh = meshes.by_name.find(mesh_name);
  if (mesh == meshes.by_name.end()) {
    return false;
  }
  if (!meshExists(mesh_name)) {
    removeMemoryMesh(mesh_name);
    return false;
  }
  return mesh->second.data_hash == data_hash && mesh->second.data_size == data_size;
}
}  // namespace

Ogre::MeshPtr loadMeshFromMemory(
  const std::string & resource_name,
  const std::string & filename,
  const std::vector<uint8_t> & data)
{
  auto & meshes = memoryMeshes();

  if (data.empty()) {
    // Retried with every message and the data may come later, so this is not logged as an error.
    auto newest = meshes.newest.find(resource_name);
    if (newest != meshes.newest.end() && meshExists(newest->second)) {
      return useMemoryMesh(newest->second);
    }
    RVIZ_RENDERING_LOG_DEBUG_STREAM(
      "Could not load embedded mesh [" << resource_name << "]: no mesh data was provided");
    return nullptr;
  }

  const size_t data_hash = hashMeshData(filename, data);
  const std::string candidates[] = {resource_name, resource_name + "#" + std::to_string(data_hash)};

  for (const auto & candidate : candidates) {
    if (holdsMemoryMesh(candidate, data_hash, data.size())) {
      return useMemoryMesh(candidate);
    }
  }

  // The extension of the file name (not of the resource name) determines the file format, and
  // with it format specific conversions such as glTF's Y-up to Z-up.
  std::string ext = std::filesystem::path(filename).extension().string();
  std::transform(
    ext.begin(), ext.end(), ext.begin(),
    [](unsigned char c) {return std::tolower(c);});
  if (ext.empty()) {
    RVIZ_RENDERING_LOG_ERROR_STREAM(
      "Could not load embedded mesh [" << resource_name << "]: file name [" << filename <<
        "] has no extension to determine the file format from");
    return nullptr;
  }

  AssimpLoader assimp_loader(nullptr);
  const aiScene * scene = assimp_loader.getSceneFromMemory(filename, data.data(), data.size());
  if (scene == nullptr) {
    RVIZ_RENDERING_LOG_ERROR_STREAM(
      "Could not load embedded mesh [" << resource_name << "] (file name [" << filename <<
        "]): " << assimp_loader.getErrorMessage());
    return nullptr;
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
    mesh = assimp_loader.meshFromAssimpScene(mesh_name, scene, ext);
  } catch (const std::exception & e) {
    RVIZ_RENDERING_LOG_ERROR_STREAM(
      "Could not load embedded mesh [" << resource_name << "]: " << e.what());
  }
  if (!mesh) {
    removeMemoryMeshResources(mesh_name);
    return nullptr;
  }

  meshes.by_name[mesh_name] = MemoryMesh{resource_name, data_hash, data.size(), 1};
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
  meshes.unused_bytes += mesh->second.data_size;
  while (meshes.unused_bytes > kMaxUnusedBytes && !meshes.unused.empty()) {
    const std::string oldest = meshes.unused.front();
    removeMemoryMesh(oldest);
  }
}

}  // namespace rviz_rendering
