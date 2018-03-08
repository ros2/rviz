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

#ifndef RVIZ_RENDERING__OBJECTS__GRID_HPP_
#define RVIZ_RENDERING__OBJECTS__GRID_HPP_

#include <cstdint>
#include <memory>
#include <vector>

#include <OgreColourValue.h>
#include <OgreMaterial.h>
#include <OgreSharedPtr.h>

#include "rviz_rendering/visibility_control.hpp"

namespace Ogre
{
class SceneManager;

class ManualObject;
class SceneNode;

class Any;
}

namespace rviz_rendering
{

class BillboardLine;

/**
 * \class Grid
 * \brief Displays a grid of cells, drawn with lines
 *
 * Displays a grid of cells, drawn with lines.  A grid with an identity orientation is drawn along the XZ plane.
 */
class Grid
{
public:
  enum Style
  {
    Lines,
    Billboards,
  };

  /**
   * \brief Constructor
   *
   * @param manager The scene manager this object is part of
   * @param cell_count The number of cells to draw
   * @param cell_length The size of each cell
   * @param line_width The line width of the cells if it is rendered in Billboards style
   * @param color The color of the lines
   */
  RVIZ_RENDERING_PUBLIC
  Grid(
    Ogre::SceneManager * manager, Ogre::SceneNode * parent_node, Style style,
    uint32_t cell_count, float cell_length, float line_width, const Ogre::ColourValue & color);
  RVIZ_RENDERING_PUBLIC
  ~Grid();

  RVIZ_RENDERING_PUBLIC
  void create();

  /**
   * \brief Get the Ogre scene node associated with this grid
   *
   * @return The Ogre scene node associated with this grid
   */
  Ogre::SceneNode * getSceneNode() {return scene_node_;}

  /**
   * \brief Sets user data on all ogre objects we own
   */
  RVIZ_RENDERING_PUBLIC
  void setUserData(const Ogre::Any & data);

  RVIZ_RENDERING_PUBLIC
  void setStyle(Style style);
  Style getStyle() {return style_;}

  RVIZ_RENDERING_PUBLIC
  void setColor(const Ogre::ColourValue & color);
  Ogre::ColourValue getColor() {return color_;}

  RVIZ_RENDERING_PUBLIC
  void setCellCount(uint32_t count);
  uint32_t getCellCount() {return cell_count_;}

  RVIZ_RENDERING_PUBLIC
  void setCellLength(float len);
  float getCellLength() {return cell_length_;}

  RVIZ_RENDERING_PUBLIC
  void setLineWidth(float width);
  float getLineWidth() {return line_width_;}

  RVIZ_RENDERING_PUBLIC
  void setHeight(uint32_t count);
  uint32_t getHeight() {return height_count_;}

  /// Exposed for testing
  Ogre::ManualObject * getManualObject() {return manual_object_;}
  std::shared_ptr<BillboardLine> getBillboardLine() {return billboard_line_;}

private:
  typedef std::function<void (const Ogre::Vector3 & p3, const Ogre::Vector3 & p4)> AddLineFunction;
  void createBillboardGrid() const;
  void createManualGrid() const;
  void createLines(AddLineFunction addLine) const;
  void createGridPlane(float extent, uint32_t height, AddLineFunction addLine) const;
  void createVerticalLinesBetweenPlanes(float extent, AddLineFunction addLine) const;
  void addBillboardLine(const Ogre::Vector3 & p3, const Ogre::Vector3 & p4) const;
  void addManualLine(const Ogre::Vector3 & p1, const Ogre::Vector3 & p2) const;
  uint32_t numberOfVerticalLines() const;

  Ogre::SceneManager * scene_manager_;
  Ogre::SceneNode * scene_node_;           ///< The scene node that this grid is attached to
  Ogre::ManualObject * manual_object_;     ///< The manual object used to draw the grid

  std::shared_ptr<BillboardLine> billboard_line_;

  Ogre::MaterialPtr material_;

  Style style_;
  uint32_t cell_count_;
  float cell_length_;
  float line_width_;
  uint32_t height_count_;
  Ogre::ColourValue color_;
};

}  // namespace rviz_rendering

#endif  // RVIZ_RENDERING__OBJECTS__GRID_HPP_
