// Copyright (c) 2010, Willow Garage, Inc.
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


#ifndef RVIZ_RENDERING__MESH_LOADER_HPP_
#define RVIZ_RENDERING__MESH_LOADER_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include <OgreMesh.h>

#include "rviz_rendering/visibility_control.hpp"

#include "resource_retriever/retriever.hpp"

namespace rviz_rendering
{

RVIZ_RENDERING_PUBLIC
Ogre::MeshPtr loadMeshFromResource(
  resource_retriever::Retriever * retriever,
  const std::string & resource_uri);

/// Load a mesh from the raw contents of a mesh file, e.g. one embedded in a message.
/**
 * The file format is determined by the extension of \p filename, which is therefore required.
 * Files referenced by the mesh (e.g. textures) are not resolved.
 *
 * The mesh is registered under \p resource_name. Loading the same data again returns the same
 * mesh, and empty \p data returns the mesh that was loaded last under \p resource_name. Different
 * data is never allowed to replace a mesh that is in use: it gets a mesh of its own, whose name
 * then differs from \p resource_name. Use Ogre::Mesh::getName() to refer to the returned mesh.
 *
 * Every successful call must be paired with a call to releaseMeshFromMemory().
 *
 * \return The mesh, or nullptr if the data is missing or could not be parsed.
 */
RVIZ_RENDERING_PUBLIC
Ogre::MeshPtr loadMeshFromMemory(
  const std::string & resource_name,
  const std::string & filename,
  const std::vector<uint8_t> & data);

/// Release a mesh obtained from loadMeshFromMemory().
/**
 * \param mesh_name The name of the mesh, as returned by Ogre::Mesh::getName().
 *
 * A mesh that is not used anymore is kept for a while, so that it can be loaded again without
 * sending the data again (e.g. after a reset). It is removed together with its materials and
 * textures once the unused meshes exceed a size limit.
 */
RVIZ_RENDERING_PUBLIC
void releaseMeshFromMemory(const std::string & mesh_name);

}  // namespace rviz_rendering

#endif  // RVIZ_RENDERING__MESH_LOADER_HPP_
