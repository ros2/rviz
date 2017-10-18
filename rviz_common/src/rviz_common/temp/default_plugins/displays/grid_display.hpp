/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

#ifndef RVIZ_COMMON__TEMP__DEFAULT_PLUGINS__DISPLAYS__GRID_DISPLAY_H_
#define RVIZ_COMMON__TEMP__DEFAULT_PLUGINS__DISPLAYS__GRID_DISPLAY_H_

#include "../../../properties/color_property.hpp"
#include "../../../properties/float_property.hpp"
#include "../../../properties/int_property.hpp"
#include "rviz_common/properties/vector_property.hpp"
#include "../../../properties/enum_property.hpp"
#include "../../../properties/tf_frame_property.hpp"
#include "../../../display.hpp"

namespace rviz_rendering
{
class Grid;
}

namespace rviz_common
{

/**
 * \class GridDisplay
 * \brief Displays a grid in either the XY, YZ, or XZ plane.
 *
 * For more information see Grid
 */
class GridDisplay : public Display
{
  Q_OBJECT

public:
  enum Plane
  {
    XY,
    XZ,
    YZ,
  };

  GridDisplay();
  virtual ~GridDisplay();

  // Overrides from Display
  virtual void onInitialize();
  virtual void update(float dt, float ros_dt);

private Q_SLOTS:
  void updateCellCount();
  void updateCellSize();
  void updateColor();
  void updateHeight();
  void updateLineWidth();
  void updateOffset();
  void updatePlane();
  void updateStyle();

private:
  rviz_rendering::Grid * grid_;            ///< Handles actually drawing the grid

  properties::TfFrameProperty * frame_property_;
  properties::IntProperty * cell_count_property_;
  properties::IntProperty * height_property_;
  properties::FloatProperty * cell_size_property_;
  properties::FloatProperty * line_width_property_;
  properties::EnumProperty * style_property_;
  properties::ColorProperty * color_property_;
  properties::FloatProperty * alpha_property_;
  properties::EnumProperty * plane_property_;
  properties::VectorProperty * offset_property_;
};

}  // namespace rviz_common

#endif  // RVIZ_COMMON__TEMP__DEFAULT_PLUGINS__DISPLAYS__GRID_DISPLAY_H_
