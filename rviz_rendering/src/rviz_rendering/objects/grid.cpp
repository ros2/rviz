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

#include "rviz_rendering/objects/grid.hpp"

#include <functional>
#include <memory>
#include <string>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreVector.h>
#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>

#include "rviz_rendering/material_manager.hpp"
#include "rviz_rendering/objects/billboard_line.hpp"

namespace rviz_rendering
{

Grid::Grid(
  Ogre::SceneManager * scene_manager,
  Ogre::SceneNode * parent_node,
  Style style,
  uint32_t cell_count,
  float cell_length,
  float line_width,
  const Ogre::ColourValue & color)
: scene_manager_(scene_manager),
  style_(style),
  cell_count_(cell_count),
  cell_length_(cell_length),
  line_width_(line_width),
  height_count_(0),
  color_(color)
{
  static uint32_t grid_count = 0;
  std::string grid_name = "Grid" + std::to_string(grid_count++);

  manual_object_ = scene_manager_->createManualObject(grid_name);

  if (!parent_node) {
    parent_node = scene_manager_->getRootSceneNode();
  }

  scene_node_ = parent_node->createChildSceneNode();
  scene_node_->attachObject(manual_object_);

  billboard_line_ = std::make_shared<rviz_rendering::BillboardLine>(scene_manager, scene_node_);

  std::string grid_material_name = grid_name + "Material";
  material_ = MaterialManager::createMaterialWithNoLighting(grid_material_name);

  setColor(color_);
}

Grid::~Grid()
{
  scene_manager_->destroySceneNode(scene_node_);
  scene_manager_->destroyManualObject(manual_object_);

  material_->unload();
}

void Grid::setCellCount(uint32_t count)
{
  cell_count_ = count;

  create();
}

void Grid::setCellLength(float len)
{
  cell_length_ = len;

  create();
}

void Grid::setLineWidth(float width)
{
  line_width_ = width;

  create();
}

void Grid::setColor(const Ogre::ColourValue & color)
{
  color_ = color;
  rviz_rendering::MaterialManager::enableAlphaBlending(material_, color.a);
  create();
}

void Grid::setStyle(Style style)
{
  style_ = style;

  create();
}

void Grid::setHeight(uint32_t height)
{
  height_count_ = height;

  create();
}

void Grid::create()
{
  manual_object_->clear();
  billboard_line_->clear();

  if (style_ == Billboards) {
    createBillboardGrid();
  } else {
    createManualGrid();
  }
}

void Grid::createManualGrid() const
{
  AddLineFunction addLineFunction = std::bind(
    &Grid::addManualLine, this, std::placeholders::_1, std::placeholders::_2);

  const uint32_t number_of_vertices_in_plane = cell_count_ * 4 * (height_count_ + 1);
  manual_object_->estimateVertexCount(number_of_vertices_in_plane + numberOfVerticalLines());

  manual_object_->begin(
    material_->getName(), Ogre::RenderOperation::OT_LINE_LIST, "rviz_rendering");
  createLines(addLineFunction);
  manual_object_->end();
}

void Grid::createBillboardGrid() const
{
  AddLineFunction addLineFunction = std::bind(
    &Grid::addBillboardLine, this, std::placeholders::_1, std::placeholders::_2);

  billboard_line_->setColor(color_.r, color_.g, color_.b, color_.a);
  billboard_line_->setLineWidth(line_width_);
  billboard_line_->setMaxPointsPerLine(2);
  const uint32_t number_of_lines_in_plane = (cell_count_ + 1) * 2 * (height_count_ + 1);
  billboard_line_->setNumLines(number_of_lines_in_plane + numberOfVerticalLines());

  createLines(addLineFunction);
}

uint32_t Grid::numberOfVerticalLines() const
{
  return (cell_count_ + 1) * (cell_count_ + 1) * height_count_;
}

void Grid::createLines(AddLineFunction addLine) const
{
  float plane_extent = (cell_length_ * static_cast<float>(cell_count_)) / 2;

  for (uint32_t current_height = 0; current_height <= height_count_; ++current_height) {
    createGridPlane(plane_extent, current_height, addLine);
  }

  if (height_count_ > 0) {
    createVerticalLinesBetweenPlanes(plane_extent, addLine);
  }
}

void Grid::createGridPlane(float extent, uint32_t height, AddLineFunction addLine) const
{
  float real_height = (height_count_ / 2.0f - static_cast<float>(height)) * cell_length_;
  for (uint32_t i = 0; i <= cell_count_; i++) {
    float inc = extent - ( i * cell_length_);

    Ogre::Vector3 p1(inc, real_height, -extent);
    Ogre::Vector3 p2(inc, real_height, extent);
    Ogre::Vector3 p3(-extent, real_height, inc);
    Ogre::Vector3 p4(extent, real_height, inc);

    addLine(p1, p2);
    addLine(p3, p4);
  }
}

void Grid::createVerticalLinesBetweenPlanes(float extent, AddLineFunction addLine) const
{
  for (uint32_t x = 0; x <= cell_count_; ++x) {
    for (uint32_t z = 0; z <= cell_count_; ++z) {
      float x_real = extent - x * cell_length_;
      float z_real = extent - z * cell_length_;

      float y_top = (height_count_ / 2.0f) * cell_length_;
      float y_bottom = -y_top;

      Ogre::Vector3 p1(x_real, y_bottom, z_real);
      Ogre::Vector3 p2(x_real, y_top, z_real);

      addLine(p1, p2);
    }
  }
}

void Grid::addManualLine(const Ogre::Vector3 & p1, const Ogre::Vector3 & p2) const
{
  manual_object_->position(p1);
  manual_object_->colour(color_);
  manual_object_->position(p2);
  manual_object_->colour(color_);
}

void Grid::addBillboardLine(const Ogre::Vector3 & p1, const Ogre::Vector3 & p2) const
{
  billboard_line_->addPoint(p1);
  billboard_line_->addPoint(p2);
  billboard_line_->finishLine();
}

void Grid::setUserData(const Ogre::Any & data)
{
  manual_object_->getUserObjectBindings().setUserAny(data);
}

}  // namespace rviz_rendering
