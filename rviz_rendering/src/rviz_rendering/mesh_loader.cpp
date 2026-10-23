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
#include <filesystem>
#include <functional>
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
// Meshes loaded from memory are named by whoever sent them, so unlike meshes loaded from a URI
// they cannot be kept forever. Each one is reference counted and removed with its materials and
// textures once it is no longer used. The hash detects new data sent under an existing name.
struct MemoryMesh
{
  size_t data_hash;
  size_t users;
};

std::unordered_map<std::string, MemoryMesh> & memoryMeshes()
{
  static std::unordered_map<std::string, MemoryMesh> meshes;
  return meshes;
}

size_t hashMeshData(const std::string & filename, const std::vector<uint8_t> & data)
{
  const size_t data_hash = std::hash<std::string_view>{}(
    std::string_view(reinterpret_cast<const char *>(data.data()), data.size()));
  return data_hash ^ (std::hash<std::string>{}(filename) << 1);
}

/// Remove a mesh created by loadMeshFromMemory, together with its materials and textures.
void removeMemoryMeshResources(const std::string & resource_name)
{
  auto & material_manager = Ogre::MaterialManager::getSingleton();
  auto & texture_manager = Ogre::TextureManager::getSingleton();
  // The assimp loader names the materials of a mesh "<resource name>Material<index>" and its
  // embedded textures "<resource name><texture name>".
  for (size_t i = 0; ; ++i) {
    const std::string material_name = resource_name + "Material" + std::to_string(i);
    Ogre::MaterialPtr material = material_manager.getByName(material_name, ROS_PACKAGE_NAME);
    if (!material) {
      break;
    }
    for (auto * technique : material->getTechniques()) {
      for (auto * pass : technique->getPasses()) {
        for (auto * texture_unit : pass->getTextureUnitStates()) {
          const std::string & texture_name = texture_unit->getTextureName();
          if (texture_name.rfind(resource_name, 0) == 0) {
            texture_manager.remove(texture_name, ROS_PACKAGE_NAME);
          }
        }
      }
    }
    material_manager.remove(material);
  }
  Ogre::MeshManager::getSingleton().remove(resource_name, ROS_PACKAGE_NAME);
}
}  // namespace

Ogre::MeshPtr loadMeshFromMemory(
  const std::string & resource_name,
  const std::string & filename,
  const std::vector<uint8_t> & data)
{
  auto & mesh_manager = Ogre::MeshManager::getSingleton();
  auto & meshes = memoryMeshes();
  auto cached = meshes.find(resource_name);
  const bool exists = cached != meshes.end() &&
    mesh_manager.resourceExists(resource_name, ROS_PACKAGE_NAME);

  // Without data, or with the data the mesh was loaded from, the cached mesh is used.
  const size_t data_hash = data.empty() ? 0 : hashMeshData(filename, data);
  if (exists && (data.empty() || cached->second.data_hash == data_hash)) {
    ++cached->second.users;
    return mesh_manager.getByName(resource_name, ROS_PACKAGE_NAME);
  }

  if (data.empty()) {
    // Retried with every message and the data may come later, so this is not logged as an error.
    RVIZ_RENDERING_LOG_DEBUG_STREAM(
      "Could not load embedded mesh [" << resource_name << "]: no mesh data was provided");
    return nullptr;
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

  // New data was sent under the name of a loaded mesh: replace it. Entities that still use the
  // old mesh keep it until they are destroyed.
  size_t users = 0;
  if (cached != meshes.end()) {
    users = cached->second.users;
    removeMemoryMeshResources(resource_name);
    meshes.erase(cached);
  }

  Ogre::MeshPtr mesh = assimp_loader.meshFromAssimpScene(resource_name, scene, ext);
  if (mesh) {
    meshes[resource_name] = MemoryMesh{data_hash, users + 1};
  } else {
    removeMemoryMeshResources(resource_name);
  }
  return mesh;
}

void releaseMeshFromMemory(const std::string & resource_name)
{
  auto & meshes = memoryMeshes();
  auto cached = meshes.find(resource_name);
  if (cached == meshes.end()) {
    return;
  }
  if (cached->second.users > 1) {
    --cached->second.users;
    return;
  }
  removeMemoryMeshResources(resource_name);
  meshes.erase(cached);
}

}  // namespace rviz_rendering
