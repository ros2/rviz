/*
 * Copyright (c) 2010, Willow Garage, Inc.
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
 *     * Neither the name of the copyright holder nor the names of its
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

#ifndef RVIZ_RENDERING__MESH_LOADER_HELPERS__ASSIMP_LOADER_HPP_
#define RVIZ_RENDERING__MESH_LOADER_HELPERS__ASSIMP_LOADER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "OgreHardwareBufferManager.h"
#include "OgreMesh.h"

#include <QDir>  // NOLINT cpplint cannot handle include order here
#include <QFileInfo>  // NOLINT cpplint cannot handle include order here
#include <QString>  // NOLINT cpplint cannot handle include order here

#define ASSIMP_UNIFIED_HEADER_NAMES 1
#if defined(ASSIMP_UNIFIED_HEADER_NAMES)
#include "assimp/Importer.hpp"
#include "assimp/scene.h"
#else
#include "assimp/aiScene.h"
#endif

#include "resource_retriever/retriever.hpp"

#include "rviz_rendering/logging.hpp"

namespace rviz_rendering
{
class AssimpLoader
{
public:
  AssimpLoader();
  Ogre::MeshPtr meshFromAssimpScene(const std::string & name, const aiScene * scene);
  const aiScene * getScene(const std::string & resource_path);
  std::string getErrorMessage();

private:
  struct SubMeshInternals
  {
    Ogre::HardwareVertexBufferSharedPtr vertex_buffer_;
    Ogre::AxisAlignedBox & axis_aligned_box_;
    float & radius_;

    SubMeshInternals(
      Ogre::HardwareVertexBufferSharedPtr vertex_buffer, Ogre::AxisAlignedBox &
      axis_aligned_box, float & radius)
    : vertex_buffer_(vertex_buffer),
      axis_aligned_box_(axis_aligned_box),
      radius_(radius)
    {}
  };

  struct MaterialInternals
  {
    Ogre::Pass * pass_;

    Ogre::ColourValue diffuse_;
    Ogre::ColourValue specular_;
    Ogre::ColourValue ambient_;

    MaterialInternals(
      Ogre::Pass * pass,
      const Ogre::ColourValue & diffuse,
      const Ogre::ColourValue & specular,
      const Ogre::ColourValue & ambient)
    : pass_(pass),
      diffuse_(diffuse),
      specular_(specular),
      ambient_(ambient)
    {}
  };

  std::vector<Ogre::MaterialPtr> loadMaterials(
    const std::string & resource_path, const aiScene * scene);
  void setLightColorsFromAssimp(
    const std::string & resource_path,
    Ogre::MaterialPtr & mat,
    const aiMaterial * ai_material,
    MaterialInternals & material_internals,
    const aiScene * ai_scene);
  void loadTexture(const std::string & resource_path);
  void loadEmbeddedTexture(
    const aiTexture * ai_texture,
    const std::string & resource_path);
  void setBlending(
    Ogre::MaterialPtr & mat, const aiMaterial * ai_material,
    const MaterialInternals & material_internals);

  void buildMesh(
    const aiScene * scene,
    const aiNode * node,
    const Ogre::MeshPtr & mesh,
    Ogre::AxisAlignedBox & axis_aligned_box,
    float & radius,
    std::vector<Ogre::MaterialPtr> & material_table);
  aiMatrix4x4 computeTransformOverSceneGraph(const aiNode * node);
  void declareVertexBufferOrdering(const aiMesh * input_mesh, const Ogre::VertexData * vertex_data);
  Ogre::HardwareVertexBufferSharedPtr allocateVertexBuffer(
    const aiMesh * input_mesh, Ogre::VertexData * vertex_data);
  void fillVertexBuffer(
    const aiMatrix4x4 & transform,
    const aiMatrix3x3 & inverse_transpose_rotation,
    const aiMesh * input_mesh,
    SubMeshInternals & internals);
  void createAndFillIndexBuffer(
    const aiMesh * input_mesh, const Ogre::SubMesh * submesh, const Ogre::VertexData * vertex_data);

  template<typename T>
  void fillIndexBuffer(const aiMesh * input_mesh, Ogre::HardwareIndexBufferSharedPtr & index_buffer)
  {
    auto * indices =
      static_cast<T *>(index_buffer->lock(Ogre::HardwareBuffer::HBL_DISCARD));

    for (T j = 0; j < input_mesh->mNumFaces; j++) {
      aiFace & face = input_mesh->mFaces[j];
      for (T k = 0; k < face.mNumIndices; ++k) {
        *indices++ = face.mIndices[k];
      }
    }
    index_buffer->unlock();
  }

  std::unique_ptr<Assimp::Importer> importer_;
};

}  // namespace rviz_rendering

#endif  // RVIZ_RENDERING__MESH_LOADER_HELPERS__ASSIMP_LOADER_HPP_
