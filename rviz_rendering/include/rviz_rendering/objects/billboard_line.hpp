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

#ifndef RVIZ_RENDERING__OBJECTS__BILLBOARD_LINE_HPP_
#define RVIZ_RENDERING__OBJECTS__BILLBOARD_LINE_HPP_

#include <cstdint>
#include <vector>

#include <OgreBillboardChain.h>
#include <OgreVector.h>
#include <OgreColourValue.h>
#include <OgreMaterial.h>
#include <OgreSharedPtr.h>

#include "rviz_rendering/objects/object.hpp"
#include "rviz_rendering/visibility_control.hpp"

namespace Ogre
{
class SceneManager;
class SceneNode;
class Quaternion;
class Any;
class BillboardChain;
}

namespace rviz_rendering
{

/**
 * \class BillboardLine
 * \brief An object that displays a multi-segment line strip rendered as billboards
 */
class BillboardLine : public Object
{
public:
  /**
   * \brief Constructor
   * @param manager Scene manager this object is a part of
   * @param parent_node A scene node to use as the parent of this object.  If NULL, uses the root scene node.
   */
  RVIZ_RENDERING_PUBLIC
  explicit BillboardLine(Ogre::SceneManager * manager, Ogre::SceneNode * parent_node = nullptr);
  RVIZ_RENDERING_PUBLIC
  ~BillboardLine() override;

  RVIZ_RENDERING_PUBLIC
  void clear();
  RVIZ_RENDERING_PUBLIC
  void finishLine();
  RVIZ_RENDERING_PUBLIC
  void addPoint(const Ogre::Vector3 & point);
  RVIZ_RENDERING_PUBLIC
  void addPoint(const Ogre::Vector3 & point, const Ogre::ColourValue & color);

  RVIZ_RENDERING_PUBLIC
  void setLineWidth(float width);

  RVIZ_RENDERING_PUBLIC
  void setMaxPointsPerLine(uint32_t max);
  RVIZ_RENDERING_PUBLIC
  void setNumLines(uint32_t num);

  // overrides from Object
  RVIZ_RENDERING_PUBLIC
  void setOrientation(const Ogre::Quaternion & orientation) override;
  RVIZ_RENDERING_PUBLIC
  void setPosition(const Ogre::Vector3 & position) override;
  RVIZ_RENDERING_PUBLIC
  void setScale(const Ogre::Vector3 & scale) override;
  RVIZ_RENDERING_PUBLIC
  void setColor(float r, float g, float b, float a) override;
  RVIZ_RENDERING_PUBLIC
  const Ogre::Vector3 & getPosition() override;
  RVIZ_RENDERING_PUBLIC
  const Ogre::Quaternion & getOrientation() override;

  /**
   * \brief Get the scene node associated with this object
   * @return The scene node associated with this object
   */
  Ogre::SceneNode * getSceneNode() {return scene_node_;}

  /**
   * \brief We have no objects that we can set user data on
   */
  void setUserData(const Ogre::Any & data) override {(void) data;}

  Ogre::MaterialPtr getMaterial() {return material_;}

  typedef std::vector<Ogre::BillboardChain *> V_ChainContainers;
  /// exposed for testing
  V_ChainContainers getChains() {return chain_containers_;}

private:
  void setupChainContainers();
  Ogre::BillboardChain * createChain();
  void changeAllElements(
    std::function<Ogre::BillboardChain::Element(Ogre::BillboardChain::Element)> change_element);
  void incrementChainContainerIfNecessary();
  void setupChainsInChainContainers() const;

  Ogre::SceneNode * scene_node_;

  V_ChainContainers chain_containers_;
  Ogre::MaterialPtr material_;

  Ogre::ColourValue color_;
  float width_;

  uint32_t num_lines_;
  uint32_t max_points_per_line_;
  uint32_t chains_per_container_;

  uint32_t current_line_;
  uint32_t current_chain_container_;
  uint32_t elements_in_current_chain_container_;
};

}  // namespace rviz_rendering

#endif  // RVIZ_RENDERING__OBJECTS__BILLBOARD_LINE_HPP_
