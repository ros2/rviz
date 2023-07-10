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

#include "assimp_loader.hpp"

#include <memory>
#include <string>
#include <vector>

#include <OgreHardwareBufferManager.h>
#include <OgreMaterial.h>
#include <OgreMaterialManager.h>
#include <OgreMeshManager.h>
#include <OgrePass.h>
#include <OgreSharedPtr.h>
#include <OgreSubMesh.h>
#include <OgreTechnique.h>
#include <OgreTechnique.h>
#include <OgreTexture.h>
#include <OgreTextureManager.h>
#include <OgreTextureUnitState.h>

#define ASSIMP_UNIFIED_HEADER_NAMES 1
#if defined(ASSIMP_UNIFIED_HEADER_NAMES)
#include <assimp/postprocess.h>
#include <assimp/IOStream.hpp>
#include <assimp/IOSystem.hpp>
#else
#include <assimp/assimp.hpp>
#include <assimp/aiPostProcess.h>
#include <assimp/IOStream.h>
#include <assimp/IOSystem.h>
#endif

namespace rviz_rendering
{

class ResourceIOStream : public Assimp::IOStream
{
public:
  explicit ResourceIOStream(const resource_retriever::MemoryResource & res)
  : res_(res),
    pos_(res.data.get())
  {}

  ~ResourceIOStream() override = default;

  size_t Read(void * buffer, size_t size, size_t count) override
  {
    size_t to_read = size * count;
    if (pos_ + to_read > res_.data.get() + res_.size) {
      to_read = res_.size - (pos_ - res_.data.get());
    }

    memcpy(buffer, pos_, to_read);
    pos_ += to_read;

    return to_read / size;
  }

  size_t Write(const void * buffer, size_t size, size_t count) override
  {
    (void) buffer;
    (void) size;
    (void) count;
    throw std::runtime_error("expected to be overridden");
  }

  aiReturn Seek(size_t offset, aiOrigin origin) override
  {
    uint8_t * new_pos = nullptr;
    switch (origin) {
      case aiOrigin_SET:
        new_pos = res_.data.get() + offset;
        break;
      case aiOrigin_CUR:
        new_pos = pos_ + offset;  // TODO(anyone): is this right? Can offset really not be negative
        break;
      case aiOrigin_END:
        new_pos = res_.data.get() + res_.size - offset;  // TODO(anyone): is this right?
        break;
      default:
        throw std::runtime_error("unexpected default in switch statement");
    }

    if (new_pos < res_.data.get() || new_pos > res_.data.get() + res_.size) {
      return aiReturn_FAILURE;
    }

    pos_ = new_pos;
    return aiReturn_SUCCESS;
  }

  size_t Tell() const override
  {
    return pos_ - res_.data.get();
  }

  size_t FileSize() const override
  {
    return res_.size;
  }

  void Flush() override
  {}

private:
  resource_retriever::MemoryResource res_;
  uint8_t * pos_;
};

class ResourceIOSystem final : public Assimp::IOSystem
{
public:
  ResourceIOSystem() = default;

  ~ResourceIOSystem() override = default;

  // Get the path delimiter character we'd like to see
  char getOsSeparator() const override
  {
    return '/';
  }

  // Check whether a specific file exists
  bool Exists(const char * file) const override
  {
    try {
      resource_retriever::MemoryResource res = retriever_.get(file);
    } catch (const resource_retriever::Exception & e) {
      (void) e;  // do nothing on exception
      return false;
    }

    return true;
  }

  // ... and finally a method to open a custom stream
  Assimp::IOStream * Open(const char * file, const char * mode = "rb") override
  {
    (void) mode;
    assert(mode == std::string("r") || mode == std::string("rb"));

    resource_retriever::MemoryResource res;
    try {
      res = retriever_.get(file);
    } catch (const resource_retriever::Exception & e) {
      (void) e;  // do nothing on exception
      return nullptr;
    }

    // This will get freed when 'Close' is called
    return new ResourceIOStream(res);
  }

  void Close(Assimp::IOStream * stream) override
  {
    delete stream;
  }

private:
  mutable resource_retriever::Retriever retriever_;
};

AssimpLoader::AssimpLoader()
{
  importer_ = std::make_unique<Assimp::Importer>();
  importer_->SetIOHandler(new ResourceIOSystem());
  // ASSIMP wants to change the orientation of the axis, but that's wrong for rviz.
  importer_->SetPropertyBool(AI_CONFIG_IMPORT_COLLADA_IGNORE_UP_DIRECTION, true);
}

Ogre::MeshPtr AssimpLoader::meshFromAssimpScene(const std::string & name, const aiScene * scene)
{
  if (!scene->HasMeshes()) {
    RVIZ_RENDERING_LOG_ERROR_STREAM("No meshes found in file [" << name.c_str() << "]");
    return Ogre::MeshPtr();
  }

  auto material_table = loadMaterials(name, scene);

  Ogre::MeshPtr mesh = Ogre::MeshManager::getSingleton().createManual(name, ROS_PACKAGE_NAME);

  Ogre::AxisAlignedBox axis_aligned_box(Ogre::AxisAlignedBox::EXTENT_NULL);
  float radius = 0.0f;
  buildMesh(scene, scene->mRootNode, mesh, axis_aligned_box, radius, material_table);

  mesh->_setBounds(axis_aligned_box);
  mesh->_setBoundingSphereRadius(radius);
  mesh->buildEdgeList();

  mesh->load();

  return mesh;
}

const aiScene * AssimpLoader::getScene(const std::string & resource_path)
{
  return importer_->ReadFile(
    resource_path,
    aiProcess_SortByPType | aiProcess_GenNormals | aiProcess_Triangulate |
    aiProcess_GenUVCoords | aiProcess_FlipUVs);
}

std::string AssimpLoader::getErrorMessage()
{
  return importer_->GetErrorString();
}

// Mostly cribbed from gazebo
/** @brief Load all materials needed by the given scene.
 * @param resource_path the path to the resource from which this scene is being loaded.
 *        loadMaterials() assumes textures for this scene are relative to the same directory that this scene is in.
 * @param scene the assimp scene to load materials for.
 * @param material_table_out Reference to the resultant material table, filled out by this function.  Is indexed the same as scene->mMaterials[].
 */
std::vector<Ogre::MaterialPtr> AssimpLoader::loadMaterials(
  const std::string & resource_path, const aiScene * scene)
{
  std::vector<Ogre::MaterialPtr> material_table_out;
  for (uint32_t i = 0; i < scene->mNumMaterials; i++) {
    std::string material_name;
    material_name = resource_path + "Material" + std::to_string(i);
    auto result = Ogre::MaterialManager::getSingleton().createOrRetrieve(
      material_name, ROS_PACKAGE_NAME, true);
    Ogre::MaterialPtr mat = std::static_pointer_cast<Ogre::Material>(result.first);
    material_table_out.push_back(mat);

    aiMaterial * ai_material = scene->mMaterials[i];

    MaterialInternals material_internals(
      mat->getTechnique(0)->getPass(0),
      Ogre::ColourValue(1.0f, 1.0f, 1.0f, 1.0f),
      Ogre::ColourValue(1.0f, 1.0f, 1.0f, 1.0f),
      Ogre::ColourValue(0, 0, 0, 1.0));


    setLightColorsFromAssimp(resource_path, mat, ai_material, material_internals, scene);

    setBlending(mat, ai_material, material_internals);

    mat->setAmbient(material_internals.ambient_ * 0.5);
    mat->setDiffuse(material_internals.diffuse_);
    material_internals.specular_.a = material_internals.diffuse_.a;
    mat->setSpecular(material_internals.specular_);
  }
  return material_table_out;
}

void AssimpLoader::setLightColorsFromAssimp(
  const std::string & resource_path,
  Ogre::MaterialPtr & mat,
  const aiMaterial * ai_material,
  MaterialInternals & material_internals,
  const aiScene * ai_scene)
{
  for (uint32_t j = 0; j < ai_material->mNumProperties; j++) {
    aiMaterialProperty * prop = ai_material->mProperties[j];
    std::string propKey = prop->mKey.data;

    if (propKey == "$tex.file") {
      aiString texture_name;
      aiTextureMapping mapping;
      uint32_t uv_index;
      ai_material->GetTexture(aiTextureType_DIFFUSE, 0, &texture_name, &mapping, &uv_index);
      std::string texture_path;
      const aiTexture * texture = ai_scene->GetEmbeddedTexture(texture_name.C_Str());
      if (texture == nullptr) {
        // It's not an embedded texture. We have to go find it.
        // Assume textures are in paths relative to the mesh
        QFileInfo resource_path_finfo(QString::fromStdString(resource_path));
        QDir resource_path_qdir = resource_path_finfo.dir();
        texture_path = resource_path_qdir.path().toStdString() + "/" + texture_name.data;
        loadTexture(texture_path);
      } else {
        // it's an embedded texture, like in GLB / glTF
        texture_path = resource_path + texture_name.data;
        loadEmbeddedTexture(texture, texture_path);
      }
      Ogre::TextureUnitState * tu = material_internals.pass_->createTextureUnitState();
      tu->setTextureName(texture_path);
    } else if (propKey == "$clr.diffuse") {
      aiColor3D clr;
      ai_material->Get(AI_MATKEY_COLOR_DIFFUSE, clr);
      material_internals.diffuse_ = Ogre::ColourValue(clr.r, clr.g, clr.b);
    } else if (propKey == "$clr.ambient") {
      aiColor3D clr;
      ai_material->Get(AI_MATKEY_COLOR_AMBIENT, clr);
      material_internals.ambient_ = Ogre::ColourValue(clr.r, clr.g, clr.b);
    } else if (propKey == "$clr.specular") {
      aiColor3D clr;
      ai_material->Get(AI_MATKEY_COLOR_SPECULAR, clr);
      material_internals.specular_ = Ogre::ColourValue(clr.r, clr.g, clr.b);
    } else if (propKey == "$clr.emissive") {
      aiColor3D clr;
      ai_material->Get(AI_MATKEY_COLOR_EMISSIVE, clr);
      mat->setSelfIllumination(clr.r, clr.g, clr.b);
    } else if (propKey == "$clr.opacity") {
      float o;
      ai_material->Get(AI_MATKEY_OPACITY, o);
      material_internals.diffuse_.a = o;
    } else if (propKey == "$mat.shininess") {
      float s;
      ai_material->Get(AI_MATKEY_SHININESS, s);
      mat->setShininess(s);
    } else if (propKey == "$mat.shadingm") {
      int model;
      ai_material->Get(AI_MATKEY_SHADING_MODEL, model);
      switch (model) {
        case aiShadingMode_Flat:
          mat->setShadingMode(Ogre::SO_FLAT);
          break;
        case aiShadingMode_Phong:
          mat->setShadingMode(Ogre::SO_PHONG);
          break;
        case aiShadingMode_Gouraud:
        default:
          mat->setShadingMode(Ogre::SO_GOURAUD);
          break;
      }
    }
  }
}

void AssimpLoader::loadEmbeddedTexture(
  const aiTexture * texture, const std::string & resource_path)
{
  if (texture == nullptr) {
    RVIZ_RENDERING_LOG_ERROR_STREAM("null texture!");
    return;
  }

  // use the format hint to try to load the image
  std::string format_hint(
    texture->achFormatHint,
    strnlen(texture->achFormatHint, sizeof(texture->achFormatHint)));

  Ogre::DataStreamPtr stream(
    new Ogre::MemoryDataStream(
      (unsigned char *)texture->pcData, texture->mWidth));

  try {
    Ogre::Image image;
    image.load(stream, format_hint.c_str());
    Ogre::TextureManager::getSingleton().loadImage(
      resource_path, ROS_PACKAGE_NAME, image);
  } catch (Ogre::Exception & e) {
    RVIZ_RENDERING_LOG_ERROR_STREAM(
      "Could not load texture [" << resource_path.c_str() <<
        "] with format hint [" << format_hint << "]: " << e.what());
  }
}

void AssimpLoader::loadTexture(const std::string & resource_path)
{
  if (!Ogre::TextureManager::getSingleton().resourceExists(resource_path, ROS_PACKAGE_NAME)) {
    resource_retriever::Retriever retriever;
    resource_retriever::MemoryResource res;
    try {
      res = retriever.get(resource_path);
    } catch (resource_retriever::Exception & e) {
      RVIZ_RENDERING_LOG_ERROR(e.what());
    }

    if (res.size != 0) {
      Ogre::DataStreamPtr stream(new Ogre::MemoryDataStream(res.data.get(), res.size));
      Ogre::Image image;
      QFileInfo resource_path_finfo(QString::fromStdString(resource_path));
      std::string extension = resource_path_finfo.completeSuffix().toStdString();

      if (extension[0] == '.') {
        extension = extension.substr(1, extension.size() - 1);
      }

      try {
        image.load(stream, extension);
        Ogre::TextureManager::getSingleton().loadImage(resource_path, ROS_PACKAGE_NAME, image);
      } catch (Ogre::Exception & e) {
        RVIZ_RENDERING_LOG_ERROR_STREAM(
          "Could not load texture [" << resource_path.c_str() << "]: " << e.what());
      }
    }
  }
}

void AssimpLoader::setBlending(
  Ogre::MaterialPtr & mat, const aiMaterial * ai_material,
  const MaterialInternals & material_internals)
{
  int mode = aiBlendMode_Default;
  ai_material->Get(AI_MATKEY_BLEND_FUNC, mode);
  switch (mode) {
    case aiBlendMode_Additive:
      mat->setSceneBlending(Ogre::SBT_ADD);
      break;
    case aiBlendMode_Default:
    default:
      {
        if (material_internals.diffuse_.a < 0.99) {
          material_internals.pass_->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
        } else {
          material_internals.pass_->setSceneBlending(Ogre::SBT_REPLACE);
        }
      }
      break;
  }
}

// Mostly stolen from gazebo
/** @brief Recursive mesh-building function.
 * @param scene is the assimp scene containing the whole mesh.
 * @param node is the current assimp node, which is part of a tree of nodes being recursed over.
 * @param material_table is indexed the same as scene->mMaterials[], and should have been filled out already by loadMaterials(). */
void AssimpLoader::buildMesh(
  const aiScene * scene,
  const aiNode * node,
  const Ogre::MeshPtr & mesh,
  Ogre::AxisAlignedBox & axis_aligned_box,
  float & radius,
  std::vector<Ogre::MaterialPtr> & material_table)
{
  if (!node) {
    return;
  }

  aiMatrix4x4 transform = computeTransformOverSceneGraph(node);

  aiMatrix3x3 rotation(transform);
  aiMatrix3x3 inverse_transpose_rotation(rotation);
  inverse_transpose_rotation.Inverse();
  inverse_transpose_rotation.Transpose();

  for (uint32_t i = 0; i < node->mNumMeshes; i++) {
    aiMesh * input_mesh = scene->mMeshes[node->mMeshes[i]];

    Ogre::SubMesh * submesh = mesh->createSubMesh();
    submesh->useSharedVertices = false;
    submesh->vertexData = new Ogre::VertexData();
    Ogre::VertexData * vertex_data = submesh->vertexData;

    declareVertexBufferOrdering(input_mesh, vertex_data);

    Ogre::HardwareVertexBufferSharedPtr vertex_buffer =
      allocateVertexBuffer(input_mesh, vertex_data);

    auto internals = SubMeshInternals(vertex_buffer, axis_aligned_box, radius);
    fillVertexBuffer(transform, inverse_transpose_rotation, input_mesh, internals);

    createAndFillIndexBuffer(input_mesh, submesh, vertex_data);

    submesh->setMaterialName(material_table[input_mesh->mMaterialIndex]->getName());
  }

  for (uint32_t i = 0; i < node->mNumChildren; ++i) {
    buildMesh(scene, node->mChildren[i], mesh, axis_aligned_box, radius, material_table);
  }
}

aiMatrix4x4 AssimpLoader::computeTransformOverSceneGraph(const aiNode * node)
{
  aiMatrix4x4 transform = node->mTransformation;
  aiNode * pnode = node->mParent;
  while (pnode) {
    transform = pnode->mTransformation * transform;
    pnode = pnode->mParent;
  }
  return transform;
}

void AssimpLoader::declareVertexBufferOrdering(
  const aiMesh * input_mesh, const Ogre::VertexData *
  vertex_data)
{
  Ogre::VertexDeclaration * vertex_decl = vertex_data->vertexDeclaration;

  size_t offset = 0;
  // positions
  vertex_decl->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
  offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);

  // normals
  if (input_mesh->HasNormals()) {
    vertex_decl->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_NORMAL);
    offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);
  }

  // texture coordinates (only support 1 for now)
  if (input_mesh->HasTextureCoords(0)) {
    vertex_decl->addElement(0, offset, Ogre::VET_FLOAT2, Ogre::VES_TEXTURE_COORDINATES, 0);
    offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT2);
  }

  // TODO(anyone): vertex colors
  (void)offset;
}

Ogre::HardwareVertexBufferSharedPtr
AssimpLoader::allocateVertexBuffer(const aiMesh * input_mesh, Ogre::VertexData * vertex_data)
{
  vertex_data->vertexCount = input_mesh->mNumVertices;
  Ogre::HardwareVertexBufferSharedPtr vertex_buffer =
    Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
    vertex_data->vertexDeclaration->getVertexSize(0),
    vertex_data->vertexCount,
    Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY,
    false);

  vertex_data->vertexBufferBinding->setBinding(0, vertex_buffer);
  return vertex_buffer;
}

void AssimpLoader::fillVertexBuffer(
  const aiMatrix4x4 & transform,
  const aiMatrix3x3 & inverse_transpose_rotation,
  const aiMesh * input_mesh,
  SubMeshInternals & internals)
{
  auto vertices = static_cast<float *>(
    internals.vertex_buffer_->lock(Ogre::HardwareBuffer::HBL_DISCARD));

  // Add the vertices
  for (uint32_t j = 0; j < input_mesh->mNumVertices; j++) {
    aiVector3D p = input_mesh->mVertices[j];
    p *= transform;
    *vertices++ = p.x;
    *vertices++ = p.y;
    *vertices++ = p.z;

    Ogre::Vector3 vector(p.x, p.y, p.z);
    internals.axis_aligned_box_.merge(vector);
    float dist = vector.length();
    if (dist > internals.radius_) {
      internals.radius_ = dist;
    }

    if (input_mesh->HasNormals()) {
      aiVector3D normal_vector = inverse_transpose_rotation * input_mesh->mNormals[j];
      normal_vector.Normalize();
      *vertices++ = normal_vector.x;
      *vertices++ = normal_vector.y;
      *vertices++ = normal_vector.z;
    }

    if (input_mesh->HasTextureCoords(0)) {
      *vertices++ = input_mesh->mTextureCoords[0][j].x;
      *vertices++ = input_mesh->mTextureCoords[0][j].y;
    }
  }

  internals.vertex_buffer_->unlock();
}

void AssimpLoader::createAndFillIndexBuffer(
  const aiMesh * input_mesh, const Ogre::SubMesh * submesh, const Ogre::VertexData * vertex_data)
{
  submesh->indexData->indexCount = 0;
  for (uint32_t j = 0; j < input_mesh->mNumFaces; j++) {
    aiFace & face = input_mesh->mFaces[j];
    submesh->indexData->indexCount += face.mNumIndices;
  }

  bool use_16_bits = vertex_data->vertexCount < (1 << 16);

  submesh->indexData->indexBuffer =
    Ogre::HardwareBufferManager::getSingleton().createIndexBuffer(
    use_16_bits ? Ogre::HardwareIndexBuffer::IT_16BIT : Ogre::HardwareIndexBuffer::IT_32BIT,
    submesh->indexData->indexCount,
    Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY,
    false);

  Ogre::HardwareIndexBufferSharedPtr index_buffer = submesh->indexData->indexBuffer;

  if (use_16_bits) {
    fillIndexBuffer<uint16_t>(input_mesh, index_buffer);
  } else {
    fillIndexBuffer<uint32_t>(input_mesh, index_buffer);
  }
}

}  // namespace rviz_rendering
