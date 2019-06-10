/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * Copyright (c) 2017, Bosch Software Innovations GmbH.
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
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
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

#ifndef RVIZ_RENDERING__MATERIAL_MANAGER_HPP_
#define RVIZ_RENDERING__MATERIAL_MANAGER_HPP_

#include <string>

#include <OgreColourValue.h>
#include <OgreMaterialManager.h>
#include <OgreSceneManager.h>

#include "rviz_rendering/visibility_control.hpp"

namespace rviz_rendering
{

const float unit_alpha_threshold = 0.9998f;

class RVIZ_RENDERING_PUBLIC MaterialManager
{
public:
  static void createColorMaterial(
    const std::string & name, const Ogre::ColourValue & color, bool use_self_illumination);

  static void createDefaultColorMaterials();

  static Ogre::MaterialPtr createMaterialWithNoLighting(std::string name);

  static Ogre::MaterialPtr createMaterialWithLighting(std::string name);

  static Ogre::MaterialPtr createMaterialWithShadowsAndLighting(std::string name);

  static Ogre::MaterialPtr createMaterialWithShadowsAndNoLighting(std::string name);

  static void createDefaultMaterials();

  static void enableAlphaBlending(Ogre::MaterialPtr material, float alpha);

  static void enableAlphaBlending(Ogre::SceneBlendType & blending, bool & depth_write, float alpha);
};

}  // namespace rviz_rendering

#endif  // RVIZ_RENDERING__MATERIAL_MANAGER_HPP_
