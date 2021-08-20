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

#include "rviz_rendering/material_manager.hpp"

#include <string>

#include <OgreMaterial.h>
#include <OgreTechnique.h>

namespace rviz_rendering
{

void MaterialManager::createColorMaterial(
  const std::string & name, const Ogre::ColourValue & color, bool use_self_illumination)
{
  Ogre::MaterialPtr mat = Ogre::MaterialManager::getSingleton().create(name, "rviz_rendering");
  mat->setAmbient(color * 0.5f);
  mat->setDiffuse(color);
  if (use_self_illumination) {
    mat->setSelfIllumination(color);
  }
  mat->setLightingEnabled(true);
  mat->setReceiveShadows(false);
}

void MaterialManager::createDefaultColorMaterials()
{
  createColorMaterial("RVIZ/Red", Ogre::ColourValue(1.0f, 0.0f, 0.0f, 1.0f), true);
  createColorMaterial("RVIZ/Green", Ogre::ColourValue(0.0f, 1.0f, 0.0f, 1.0f), true);
  createColorMaterial("RVIZ/Blue", Ogre::ColourValue(0.0f, 0.0f, 1.0f, 1.0f), true);
  createColorMaterial("RVIZ/Cyan", Ogre::ColourValue(0.0f, 1.0f, 1.0f, 1.0f), true);
  createColorMaterial("RVIZ/ShadedRed", Ogre::ColourValue(1.0f, 0.0f, 0.0f, 1.0f), false);
  createColorMaterial("RVIZ/ShadedGreen", Ogre::ColourValue(0.0f, 1.0f, 0.0f, 1.0f), false);
  createColorMaterial("RVIZ/ShadedBlue", Ogre::ColourValue(0.0f, 0.0f, 1.0f, 1.0f), false);
  createColorMaterial("RVIZ/ShadedCyan", Ogre::ColourValue(0.0f, 1.0f, 1.0f, 1.0f), false);
}

Ogre::MaterialPtr MaterialManager::createMaterialWithNoLighting(std::string name)
{
  Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create(name, "rviz_rendering");
  material->setReceiveShadows(false);
  material->getTechnique(0)->setLightingEnabled(false);

  return material;
}

Ogre::MaterialPtr MaterialManager::createMaterialWithLighting(std::string name)
{
  Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create(name, "rviz_rendering");
  material->setReceiveShadows(false);
  material->getTechnique(0)->setLightingEnabled(true);

  return material;
}

Ogre::MaterialPtr MaterialManager::createMaterialWithShadowsAndLighting(std::string name)
{
  Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create(name, "rviz_rendering");
  material->getTechnique(0)->setLightingEnabled(true);

  return material;
}

Ogre::MaterialPtr MaterialManager::createMaterialWithShadowsAndNoLighting(std::string name)
{
  Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create(name, "rviz_rendering");
  material->getTechnique(0)->setLightingEnabled(false);

  return material;
}

void MaterialManager::enableAlphaBlending(Ogre::MaterialPtr material, float alpha)
{
  if (alpha < unit_alpha_threshold) {
    material->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    material->setDepthWriteEnabled(false);
  } else {
    material->setSceneBlending(Ogre::SBT_REPLACE);
    material->setDepthWriteEnabled(true);
  }
}

void MaterialManager::enableAlphaBlending(
  Ogre::SceneBlendType & blending, bool & depth_write, float alpha)
{
  if (alpha < unit_alpha_threshold) {
    blending = Ogre::SBT_TRANSPARENT_ALPHA;
    depth_write = false;
  } else {
    blending = Ogre::SBT_REPLACE;
    depth_write = true;
  }
}

void MaterialManager::createDefaultMaterials()
{
  auto retrieve_result = Ogre::MaterialManager::getSingleton().createOrRetrieve(
    "BaseWhiteNoLighting", "rviz_rendering");
  auto material = std::dynamic_pointer_cast<Ogre::Material>(retrieve_result.first);
  if (material) {
    material->setLightingEnabled(false);
  }
}

}  // namespace rviz_rendering
