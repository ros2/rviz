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

#include "rviz_rendering/objects/billboard_line.hpp"

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreVector.h>
#include <OgreQuaternion.h>
#include <OgreBillboardChain.h>
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>

#include <iostream>
#include <string>

#include "rviz_rendering/material_manager.hpp"

static const uint32_t MAX_ELEMENTS = (65536 / 4);

namespace rviz_rendering
{

BillboardLine::BillboardLine(Ogre::SceneManager * scene_manager, Ogre::SceneNode * parent_node)
: Object(scene_manager),
  width_(0.1f),
  num_lines_(1),
  max_points_per_line_(100),
  chains_per_container_(0),
  current_line_(0),
  current_chain_container_(0),
  elements_in_current_chain_container_(0)
{
  if (!parent_node) {
    parent_node = scene_manager_->getRootSceneNode();
  }

  scene_node_ = parent_node->createChildSceneNode();

  static int count = 0;
  std::string material_name = "BillboardLineMaterial" + std::to_string(count++);
  material_ = MaterialManager::createMaterialWithNoLighting(material_name);

  setNumLines(num_lines_);
  setMaxPointsPerLine(max_points_per_line_);
}

BillboardLine::~BillboardLine()
{
  for (auto & chain : chain_containers_) {
    scene_manager_->destroyBillboardChain(chain);
  }

  scene_manager_->destroySceneNode(scene_node_);

  Ogre::MaterialManager::getSingleton().remove(material_);
}

void BillboardLine::clear()
{
  for (auto & chain : chain_containers_) {
    chain->clearAllChains();
  }

  current_line_ = 0;
  current_chain_container_ = 0;
  elements_in_current_chain_container_ = 0;
}

void BillboardLine::setupChainContainers()
{
  uint32_t total_points = max_points_per_line_ * num_lines_;
  uint32_t num_chains = total_points / MAX_ELEMENTS;
  if (total_points % MAX_ELEMENTS != 0) {
    ++num_chains;
  }

  for (uint32_t i = static_cast<uint32_t>(chain_containers_.size()); i < num_chains; ++i) {
    createChain();
  }

  chains_per_container_ = max_points_per_line_ > 0 ? MAX_ELEMENTS / max_points_per_line_ : 1;
  if (max_points_per_line_ > MAX_ELEMENTS) {
    chains_per_container_ = 1;
  }

  setupChainsInChainContainers();
}

Ogre::BillboardChain * BillboardLine::createChain()
{
  std::stringstream ss;
  static int count = 0;
  ss << "BillboardLine chain" << count++;
  Ogre::BillboardChain * chain = scene_manager_->createBillboardChain(ss.str());
  chain->setMaterialName(material_->getName());
  scene_node_->attachObject(chain);

  chain_containers_.push_back(chain);

  return chain;
}

void BillboardLine::setupChainsInChainContainers() const
{
  auto it = chain_containers_.begin();
  auto end = chain_containers_.end();
  for (; it != end; ++it) {
    (*it)->setMaxChainElements(max_points_per_line_);

    // shorten the number of chains in the last bbchain, to avoid memory wasteage
    if (it + 1 == end) {
      uint32_t lines_left = num_lines_ % chains_per_container_;

      // Handle the case where num_lines_ is a multiple of lines_per_chain
      (*it)->setNumberOfChains((lines_left == 0) ? chains_per_container_ : lines_left);
    } else {
      (*it)->setNumberOfChains(chains_per_container_);
    }
  }
}

void BillboardLine::setMaxPointsPerLine(uint32_t max)
{
  max_points_per_line_ = max;

  setupChainContainers();
}

void BillboardLine::setNumLines(uint32_t num)
{
  num_lines_ = num;

  setupChainContainers();
}

void BillboardLine::finishLine()
{
  ++current_line_;

  assert(current_line_ <= num_lines_);
}

void BillboardLine::addPoint(const Ogre::Vector3 & point)
{
  addPoint(point, color_);
}

void BillboardLine::addPoint(const Ogre::Vector3 & point, const Ogre::ColourValue & color)
{
  assert(current_line_ < num_lines_);
  assert(
    chain_containers_[current_chain_container_]->
    getNumChainElements(current_line_ % chains_per_container_) <= max_points_per_line_);

  incrementChainContainerIfNecessary();

  rviz_rendering::MaterialManager::enableAlphaBlending(material_, color.a);

  Ogre::BillboardChain::Element e;
  e.position = point;
  e.width = width_;
  e.colour = color;
  chain_containers_[current_chain_container_]->addChainElement(
    current_line_ % chains_per_container_, e);
}

void BillboardLine::incrementChainContainerIfNecessary()
{
  ++elements_in_current_chain_container_;
  if (elements_in_current_chain_container_ > MAX_ELEMENTS) {
    ++current_chain_container_;
    elements_in_current_chain_container_ = 1;
  }
}

void BillboardLine::setLineWidth(float width)
{
  width_ = width;

  changeAllElements(
    [width](Ogre::BillboardChain::Element element) {
      element.width = width;
      return element;
    });
}

void BillboardLine::setPosition(const Ogre::Vector3 & position)
{
  scene_node_->setPosition(position);
}

void BillboardLine::setOrientation(const Ogre::Quaternion & orientation)
{
  scene_node_->setOrientation(orientation);
}

void BillboardLine::setScale(const Ogre::Vector3 & scale)
{
  // Setting scale doesn't really make sense here
  (void) scale;
}

void BillboardLine::setColor(float r, float g, float b, float a)
{
  rviz_rendering::MaterialManager::enableAlphaBlending(material_, a);

  color_ = Ogre::ColourValue(r, g, b, a);

  changeAllElements(
    [this](Ogre::BillboardChain::Element element) {
      element.colour = color_;
      return element;
    });
}

void BillboardLine::changeAllElements(
  std::function<Ogre::BillboardChain::Element(Ogre::BillboardChain::Element)> change_element)
{
  for (uint32_t line = 0; line < num_lines_; ++line) {
    Ogre::BillboardChain * container = chain_containers_[line / chains_per_container_];
    uint32_t chain_index = line % chains_per_container_;
    size_t elements_in_chain = container->getNumChainElements(chain_index);

    for (uint32_t i = 0; i < elements_in_chain; ++i) {
      Ogre::BillboardChain::Element element = container->getChainElement(chain_index, i);
      Ogre::BillboardChain::Element new_element = change_element(element);
      container->updateChainElement(chain_index, i, new_element);
    }
  }
}

const Ogre::Vector3 & BillboardLine::getPosition()
{
  return scene_node_->getPosition();
}

const Ogre::Quaternion & BillboardLine::getOrientation()
{
  return scene_node_->getOrientation();
}

}  // namespace rviz_rendering
