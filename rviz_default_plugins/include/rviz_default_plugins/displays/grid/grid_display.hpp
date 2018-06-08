/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * Copyright (c) 2017, Open Source Robotics Foundation, Inc.
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

#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__GRID__GRID_DISPLAY_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__GRID__GRID_DISPLAY_HPP_

#include <memory>

#include "rviz_common/display.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/int_property.hpp"
#include "rviz_common/properties/tf_frame_property.hpp"
#include "rviz_common/properties/vector_property.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

namespace rviz_rendering
{

class Grid;

}  // namespace rviz_rendering

namespace rviz_default_plugins
{
namespace displays
{

/**
 * \class GridDisplay
 * \brief Displays a grid in either the XY, YZ, or XZ plane.
 *
 * For more information see Grid
 */
class RVIZ_DEFAULT_PLUGINS_PUBLIC GridDisplay : public rviz_common::Display
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
  ~GridDisplay() override;

  // Overrides from Display
  void onInitialize() override;
  void update(float dt, float ros_dt) override;

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
  std::unique_ptr<rviz_rendering::Grid> grid_;  ///< Handles actually drawing the grid

  rviz_common::properties::TfFrameProperty * frame_property_;
  rviz_common::properties::IntProperty * cell_count_property_;
  rviz_common::properties::IntProperty * height_property_;
  rviz_common::properties::FloatProperty * cell_size_property_;
  rviz_common::properties::FloatProperty * line_width_property_;
  rviz_common::properties::EnumProperty * style_property_;
  rviz_common::properties::ColorProperty * color_property_;
  rviz_common::properties::FloatProperty * alpha_property_;
  rviz_common::properties::EnumProperty * plane_property_;
  rviz_common::properties::VectorProperty * offset_property_;
};

}  // namespace displays
}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__GRID__GRID_DISPLAY_HPP_
